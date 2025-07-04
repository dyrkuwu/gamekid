import sys
import argparse
from typing import Dict, List, Tuple

class GKSMCompiler:
    def __init__(self):
        self.instructions = {
            'nop': (0x00, 0),
            'addrm': (0x10, 2), 'addmr': (0x11, 2), 'addrr': (0x12, 2), 'addmm': (0x13, 2),
            'subrm': (0x20, 2), 'submr': (0x21, 2), 'subrr': (0x22, 2), 'submm': (0x23, 2),
            'mulrm': (0x30, 2), 'mulmr': (0x31, 2), 'mulrr': (0x32, 2), 'mulmm': (0x33, 2),
            'divrm': (0x40, 2), 'divmr': (0x41, 2), 'divrr': (0x42, 2), 'divmm': (0x43, 2),
            'andrm': (0x50, 2), 'andmr': (0x51, 2), 'andrr': (0x52, 2), 'andmm': (0x53, 2),
            'orrm': (0x60, 2), 'ormr': (0x61, 2), 'orrr': (0x62, 2), 'ormm': (0x63, 2),
            'ldr': (0x70, 2), 'ldm': (0x71, 2), 'ldrm': (0x72, 2), 'ldmr': (0x73, 2),
            'ldrr': (0x74, 2), 'ldmm': (0x75, 2), 'ldri': (0x76, 2), 'ldir': (0x77, 2),
            'ldrio': (0x78, 3), 'ldior': (0x79, 3),
            'push': (0x80, 1), 'pop': (0x90, 1),
            'jmp': (0xA0, 1), 'jz': (0xA1, 1), 'jnz': (0xA2, 1), 'jc': (0xA3, 1), 'jnc': (0xA4, 1),
            'cmprm': (0xB0, 2), 'cmpmr': (0xB1, 2), 'cmprr': (0xB2, 2), 'cmpmm': (0xB3, 2),
            'incr': (0xC0, 1), 'decr': (0xC1, 1), 'incm': (0xC2, 1), 'decm': (0xC3, 1),
            'shlr': (0xD0, 2), 'shrr': (0xD1, 2), 'shlm': (0xD2, 2), 'shrm': (0xD3, 2),
            'ldrai': (0xE0, 2), 'strai': (0xE1, 2),
            'swprr': (0xF0, 2), 'swpmm': (0xF1, 2), 'movzr': (0xF2, 2), 'movnzr': (0xF3, 2),
            'movzm': (0xF4, 2), 'movnzm': (0xF5, 2),
            'halt': (0xAA, 0)
        }
        
        self.labels: Dict[str, int] = {}
        self.unresolved_labels: List[Tuple[int, str, str]] = []
        self.output: List[int] = []
        self.current_address = 0x0100
        
    def parse_value(self, value_str: str) -> int:
        value_str = value_str.strip()
        
        if value_str.startswith('0x') or value_str.startswith('0X'):
            value = int(value_str, 16)
        elif value_str.startswith('r'):
            reg_num = int(value_str[1:])
            if reg_num < 0 or reg_num > 19:
                raise ValueError(f"Invalid register: {value_str}")
            return reg_num
        elif value_str.isdigit():
            value = int(value_str)
        else:
            value = int(value_str)
        
        if value < 0:
            raise ValueError(f"Value cannot be negative: {value_str}")
        
        return value
    
    def parse_address(self, addr_str: str) -> Tuple[int, int]:
        addr = self.parse_value(addr_str)
        if addr < 0 or addr > 0xFFFF:
            raise ValueError(f"Address out of range: {addr_str}")
        return (addr & 0xFF, (addr >> 8) & 0xFF)
    
    def parse_line(self, line: str, line_num: int) -> None:
        line = line.strip()
        
        if not line or line.startswith(';'):
            return
            
        comment_pos = line.find(';')
        if comment_pos != -1:
            line = line[:comment_pos].strip()
            
        if not line:
            return
            
        if line.endswith(':'): # check if this is a label
            label = line[:-1].strip().lower()
            self.labels[label] = self.current_address
            return
            
        parts = line.split()
        if not parts:
            return
            
        mnemonic = parts[0].lower()
        
        if mnemonic not in self.instructions:
            raise ValueError(f"Line {line_num}: Unknown instruction '{mnemonic}'")
            
        opcode, arg_count = self.instructions[mnemonic]
        
        if len(parts) - 1 != arg_count:
            raise ValueError(f"Line {line_num}: Instruction '{mnemonic}' expects {arg_count} arguments, got {len(parts) - 1}")
        
        self.output.append(opcode)
        self.current_address += 1
        
        args = [arg.rstrip(',') for arg in parts[1:]]
        self.parse_instruction_args(mnemonic, args, line_num)
    
    def parse_instruction_args(self, mnemonic: str, args: List[str], line_num: int) -> None:
        try:
            if mnemonic == 'ldm':
                addr = self.parse_value(args[0])
                value = self.parse_value(args[1])
                if value > 255:
                    raise ValueError(f"Value {value} too large for byte")
                addr_low = addr & 0xFF
                addr_high = (addr >> 8) & 0xFF
                self.output.extend([addr_low, addr_high, value])
                self.current_address += 3
            
            elif mnemonic in ['jmp', 'jz', 'jnz', 'jc', 'jnc']:
                if args[0].startswith('0x') or args[0].startswith('0X'):
                    addr = self.parse_value(args[0])
                    self.output.extend([addr & 0xFF, (addr >> 8) & 0xFF])
                    self.current_address += 2
                else:
                    self.unresolved_labels.append((len(self.output), args[0].lower(), 'addr16'))
                    self.output.extend([0, 0])
                    self.current_address += 2
            
            elif mnemonic in ['ldr', 'ldrr', 'addrr', 'subrr', 'mulrr', 'divrr', 'andrr', 'orrr', 'cmprr', 'swprr', 'movzr', 'movnzr']:
                for arg in args:
                    value = self.parse_value(arg)
                    if mnemonic == 'ldr' and len(args) == 2 and arg == args[1]: # second arg of ldr is immediate value
                        if value > 255:
                            raise ValueError(f"Immediate value {value} too large for byte")
                    self.output.append(value)
                    self.current_address += 1
            
            elif mnemonic in ['push', 'pop', 'incr', 'decr']:
                value = self.parse_value(args[0])
                self.output.append(value)
                self.current_address += 1
            
            elif mnemonic in ['ldri', 'ldir', 'ldrai', 'strai']:
                for arg in args:
                    value = self.parse_value(arg)
                    self.output.append(value)
                    self.current_address += 1
            
            elif mnemonic in ['shlr', 'shrr']:
                for arg in args:
                    value = self.parse_value(arg)
                    self.output.append(value)
                    self.current_address += 1
            
            elif mnemonic in ['incm', 'decm']:
                addr = self.parse_value(args[0])
                self.output.extend([addr & 0xFF, (addr >> 8) & 0xFF])
                self.current_address += 2
            
            elif mnemonic in ['addrm', 'subrm', 'mulrm', 'divrm', 'andrm', 'orrm', 'cmprm']:
                reg = self.parse_value(args[0])
                addr = self.parse_value(args[1])
                self.output.extend([reg, addr & 0xFF, (addr >> 8) & 0xFF])
                self.current_address += 3
            
            elif mnemonic in ['addmr', 'submr', 'mulmr', 'divmr', 'andmr', 'ormr', 'cmpmr']:
                reg = self.parse_value(args[0])
                addr = self.parse_value(args[1])
                self.output.extend([reg, addr & 0xFF, (addr >> 8) & 0xFF])
                self.current_address += 3
            
            elif mnemonic in ['ldrm']:
                reg = self.parse_value(args[0])
                addr = self.parse_value(args[1])
                self.output.extend([reg, addr & 0xFF, (addr >> 8) & 0xFF])
                self.current_address += 3
            
            elif mnemonic in ['ldmr']:
                addr = self.parse_value(args[0])
                reg = self.parse_value(args[1])
                self.output.extend([addr & 0xFF, (addr >> 8) & 0xFF, reg])
                self.current_address += 3
            
            elif mnemonic in ['ldrio']:
                reg = self.parse_value(args[0])
                addr_reg = self.parse_value(args[1])
                offset = self.parse_value(args[2])
                self.output.extend([reg, addr_reg, offset])
                self.current_address += 3
            
            elif mnemonic in ['ldior']:
                addr_reg = self.parse_value(args[0])
                offset = self.parse_value(args[1])
                reg = self.parse_value(args[2])
                self.output.extend([addr_reg, offset, reg])
                self.current_address += 3
            
            elif mnemonic in ['shlm', 'shrm']:
                addr = self.parse_value(args[0])
                amount = self.parse_value(args[1])
                self.output.extend([addr & 0xFF, (addr >> 8) & 0xFF, amount])
                self.current_address += 3
            
            elif mnemonic in ['movzm', 'movnzm']:
                addr = self.parse_value(args[0])
                reg = self.parse_value(args[1])
                self.output.extend([addr & 0xFF, (addr >> 8) & 0xFF, reg])
                self.current_address += 3
            
            elif mnemonic in ['addmm', 'submm', 'mulmm', 'divmm', 'andmm', 'ormm', 'ldmm', 'cmpmm']:
                addr1 = self.parse_value(args[0])
                addr2 = self.parse_value(args[1])
                self.output.extend([addr1 & 0xFF, (addr1 >> 8) & 0xFF, addr2 & 0xFF, (addr2 >> 8) & 0xFF])
                self.current_address += 4
            
            elif mnemonic in ['swpmm']:
                addr1 = self.parse_value(args[0])
                addr2 = self.parse_value(args[1])
                self.output.extend([addr1 & 0xFF, (addr1 >> 8) & 0xFF, addr2 & 0xFF, (addr2 >> 8) & 0xFF])
                self.current_address += 4
            
        except ValueError as e:
            raise ValueError(f"Line {line_num}: {e}")
        except IndexError:
            raise ValueError(f"Line {line_num}: Not enough arguments for instruction '{mnemonic}'")
    
    def resolve_labels(self) -> None:
        for pos, label, label_type in self.unresolved_labels:
            if label not in self.labels:
                raise ValueError(f"Undefined label: {label}")
                
            addr = self.labels[label]
            
            if label_type == 'addr16':
                self.output[pos] = addr & 0xFF
                self.output[pos + 1] = (addr >> 8) & 0xFF
            else:
                self.output[pos] = addr & 0xFF
    
    def compile(self, source_code: str) -> bytes:
        lines = source_code.split('\n')
        
        for line_num, line in enumerate(lines, 1):
            self.parse_line(line, line_num)
        
        self.resolve_labels()
        
        return bytes(self.output)

def main():
    parser = argparse.ArgumentParser(description='GKSM Assembly Compiler')
    parser.add_argument('input_file', help='Input GKSM assembly file')
    parser.add_argument('-o', '--output', help='Output ROM file (default: input_file.rom)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    if args.output is None:
        if args.input_file.endswith('.gksm'):
            args.output = args.input_file[:-5] + '.rom'
        else:
            args.output = args.input_file + '.rom'
    
    try:
        with open(args.input_file, 'r') as f:
            source_code = f.read()
        
        compiler = GKSMCompiler()
        rom_data = compiler.compile(source_code)
        
        with open(args.output, 'wb') as f:
            f.write(rom_data)
        
        if args.verbose:
            print(f"Compiled {args.input_file} to {args.output}")
            print(f"Output size: {len(rom_data)} bytes")
            print(f"Labels found: {list(compiler.labels.keys())}")
        
        print(f"Successfully compiled {args.input_file} -> {args.output}")
        
    except FileNotFoundError:
        print(f"Error: File '{args.input_file}' not found", file=sys.stderr)
        sys.exit(1)
    except ValueError as e:
        print(f"Compilation error: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main() 