use clap::Parser;
use pixels::{Error, Pixels, SurfaceTexture};
use winit::dpi::LogicalSize;
use winit::event::{Event, WindowEvent};
use winit::event_loop::EventLoop;
use winit::keyboard::KeyCode;
use winit::window::{WindowBuilder, WindowButtons};
use winit_input_helper::WinitInputHelper;

const SCREEN_WIDTH: u32 = 16;
const SCREEN_HEIGHT: u32 = 17;
const WINDOW_WIDTH: u32 = 256;
const WINDOW_HEIGHT: u32 = 272;

const PC_LOW_ADDR: u16 = 2041;
const PC_HIGH_ADDR: u16 = 2042;
const SP_LOW_ADDR: u16 = 2043;
const SP_HIGH_ADDR: u16 = 2044;
const FLAGS_ADDR: u16 = 2045;
const KEY_INPUT_ADDR: u16 = 2046;
const RANDOM_ADDR: u16 = 2047;
const REG_BASE_ADDR: u16 = 2021;
const STACK_BASE_LOW: u8 = 0xF0;
const STACK_BASE_HIGH: u8 = 0x05;
const PROG_BASE_LOW: u8 = 0;
const PROG_BASE_HIGH: u8 = 1;

const KEY_UP: u8 = 0x01;
const KEY_DOWN: u8 = 0x02;
const KEY_LEFT: u8 = 0x04;
const KEY_RIGHT: u8 = 0x08;
const KEY_A: u8 = 0x16;
const KEY_B: u8 = 0x32;

const FLAG_ZERO: u8 = 0x01;
const FLAG_CARRY: u8 = 0x02;

#[derive(Parser)]
#[command(name = "gamekid")]
#[command(about = "An emulator for the (non existent) GameKid handheld")]
struct Args {
    #[arg(long, help = "ROM file to load")]
    rom: Option<String>,

    #[arg(long, help = "Show instruction debug info")]
    debug: bool,

    #[arg(long, help = "Corrupt memory")]
    corrupt: bool,
}

/*
 * mem docs:
 * 0x0000-0x00FF (0-255): video memory - 16x16 screen (256 bytes)
 * 0x0100-0x05EF (256-1519): program memory (1264 bytes)
 * 0x05F0-0x07E4 (1520-2020): stack memory (501 bytes)
 * 0x07E5-0x07F8 (2021-2040): registers r0-r19 (20 bytes)
 * 0x07F9-0x07FA (2041-2042): pc low/high bytes
 * 0x07FB-0x07FC (2043-2044): sp low/high bytes
 * 0x07FD (2045): flags (1 byte)
 * 0x07FE (2046): key input (1 byte)
 * 0x07FF (2047): random byte (1 byte)
*/

/*
 * instructions:
 * 0x00 - nop
 * 0x10 - addrm reg addr_low addr_high       (add register to memory at address)
 * 0x11 - addmr reg addr_low addr_high       (add memory at address to register)
 * 0x12 - addrr reg1 reg2                    (add reg2 to reg1, store in reg1)
 * 0x13 - addmm addr1_low addr1_high addr2_low addr2_high (add mem2 to mem1, store in mem1)
 * 0x20 - subrm reg addr_low addr_high       (subtract register from memory at address)
 * 0x21 - submr reg addr_low addr_high       (subtract memory at address from register)
 * 0x22 - subrr reg1 reg2                    (subtract reg2 from reg1, store in reg1)
 * 0x23 - submm addr1_low addr1_high addr2_low addr2_high (subtract mem2 from mem1, store in mem1)
 * 0x30 - mulrm reg addr_low addr_high       (multiply register by memory at address, store in register)
 * 0x31 - mulmr reg addr_low addr_high       (multiply memory at address by register, store in memory)
 * 0x32 - mulrr reg1 reg2                    (multiply reg1 by reg2, store in reg1)
 * 0x33 - mulmm addr1_low addr1_high addr2_low addr2_high (multiply mem1 by mem2, store in mem1)
 * 0x40 - divrm reg addr_low addr_high       (divide register by memory at address, store in register)
 * 0x41 - divmr reg addr_low addr_high       (divide memory at address by register, store in memory)
 * 0x42 - divrr reg1 reg2                    (divide reg1 by reg2, store in reg1)
 * 0x43 - divmm addr1_low addr1_high addr2_low addr2_high (divide mem1 by mem2, store in mem1)
 * 0x50 - andrm reg addr_low addr_high       (bitwise AND register with memory at address, store in register)
 * 0x51 - andmr reg addr_low addr_high       (bitwise AND memory at address with register, store in memory)
 * 0x52 - andrr reg1 reg2                    (bitwise AND reg1 with reg2, store in reg1)
 * 0x53 - andmm addr1_low addr1_high addr2_low addr2_high (bitwise AND mem1 with mem2, store in mem1)
 * 0x60 - orrm  reg addr_low addr_high       (bitwise OR register with memory at address, store in register)
 * 0x61 - ormr  reg addr_low addr_high       (bitwise OR memory at address with register, store in memory)
 * 0x62 - orrr  reg1 reg2                    (bitwise OR reg1 with reg2, store in reg1)
 * 0x63 - ormm  addr1_low addr1_high addr2_low addr2_high (bitwise OR mem1 with mem2, store in mem1)
 * 0x70 - ldr   reg value                    (load immediate value into register)
 * 0x71 - ldm   addr_low addr_high value     (load immediate value into memory at address)
 * 0x72 - ldrm  reg addr_low addr_high       (load memory value into register)
 * 0x73 - ldmr  addr_low addr_high reg       (load register value into memory)
 * 0x74 - ldrr  reg1 reg2                    (load reg2 value into reg1)
 * 0x75 - ldmm  addr1_low addr1_high addr2_low addr2_high (load mem2 value into mem1)
 * 0x76 - ldri  reg addr_reg                 (load from address in addr_reg into reg)
 * 0x77 - ldir  addr_reg reg                 (store reg into address stored in addr_reg)
 * 0x78 - ldrio reg addr_reg offset          (load from address in addr_reg + offset into reg)
 * 0x79 - ldior addr_reg offset reg          (store reg into address in addr_reg + offset)
 * 0x80 - push  reg                          (push register to stack)
 * 0x90 - pop   reg                          (pop stack to register)
 * 0xA0 - jmp   addr_low addr_high           (unconditional jump to address)
 * 0xA1 - jz    addr_low addr_high           (jump if zero flag set)
 * 0xA2 - jnz   addr_low addr_high           (jump if zero flag not set)
 * 0xA3 - jc    addr_low addr_high           (jump if carry flag set)
 * 0xA4 - jnc   addr_low addr_high           (jump if carry flag not set)
 * 0xB0 - cmprm reg addr_low addr_high       (compare register with memory, set flags)
 * 0xB1 - cmpmr reg addr_low addr_high       (compare memory with register, set flags)
 * 0xB2 - cmprr reg1 reg2                    (compare reg1 with reg2, set flags)
 * 0xB3 - cmpmm addr1_low addr1_high addr2_low addr2_high (compare mem1 with mem2, set flags)
 * 0xC0 - incr  reg                          (increment register)
 * 0xC1 - decr  reg                          (decrement register)
 * 0xC2 - incm  addr_low addr_high           (increment memory at address)
 * 0xC3 - decm  addr_low addr_high           (decrement memory at address)
 * 0xD0 - shlr  reg amount                   (shift left register by amount)
 * 0xD1 - shrr  reg amount                   (shift right register by amount)
 * 0xD2 - shlm  addr_low addr_high amount    (shift left memory by amount)
 * 0xD3 - shrm  addr_low addr_high amount    (shift right memory by amount)
 * 0xE0 - ldrai reg addr_reg                 (load from addr_reg into reg, then increment addr_reg)
 * 0xE1 - strai addr_reg reg                 (store reg into addr_reg, then increment addr_reg)
 * 0xF0 - swprr reg1 reg2                    (swap two registers)
 * 0xF1 - swpmm addr1_low addr1_high addr2_low addr2_high (swap two memory locations)
 * 0xF2 - movzr reg1 reg2                    (move reg2 to reg1 if zero flag set)
 * 0xF3 - movnzr reg1 reg2                   (move reg2 to reg1 if zero flag not set)
 * 0xF4 - movzm addr_low addr_high reg       (move reg to memory if zero flag set)
 * 0xF5 - movnzm addr_low addr_high reg      (move reg to memory if zero flag not set)
 * 0xAA - halt                               (stop execution)
*/

#[rustfmt::skip]
const FALLBACK_ROM: [u8; 35] = [
    0x70, 0x00, 0x00,    // 0x0100: ldr r0, 0      - address = 0 
    0x70, 0x01, 0x00,    // 0x0103: ldr r1, 0      - next_col = 0
    
    0xC0, 0x01,          // 0x0106: incr r1        - ++next_col (LOOP START)
    0x70, 0x02, 0x10,    // 0x0108: ldr r2, 16     - load 16 for comparison
    0xB2, 0x01, 0x02,    // 0x010B: cmprr r1, r2   - compare next_col with 16
    0xA2, 0x14, 0x01,    // 0x010E: jnz 0x0114     - if (next_col != 16) skip reset
    0x70, 0x01, 0x00,    // 0x0111: ldr r1, 0      - next_col = 0 (reset)
    
    0x77, 0x00, 0x01,    // 0x0114: ldir r0, r1    - vmem[address] = next_col (SKIP_RESET)
    0xC0, 0x00,          // 0x0117: incr r0        - address++ 
    0x70, 0x02, 0x00,    // 0x0119: ldr r2, 0      - load 0 for comparison
    0xB2, 0x00, 0x02,    // 0x011C: cmprr r0, r2   - compare address with 0
    0xA2, 0x06, 0x01,    // 0x011F: jnz 0x0106     - if (address != 0) continue loop
    0xAA                 // 0x0122: halt           - done (address wrapped from 255 to 0)
];

fn read_u16(mem: &[u8; 2048], low_addr: u16, high_addr: u16) -> u16 {
    (mem[low_addr as usize] as u16) | ((mem[high_addr as usize] as u16) << 8)
}

fn write_u16(mem: &mut [u8; 2048], low_addr: u16, high_addr: u16, value: u16) {
    mem[low_addr as usize] = (value & 0xFF) as u8;
    mem[high_addr as usize] = (value >> 8) as u8;
}

fn get_pc(mem: &[u8; 2048]) -> u16 {
    read_u16(mem, PC_LOW_ADDR, PC_HIGH_ADDR)
}

fn set_pc(mem: &mut [u8; 2048], value: u16) {
    write_u16(mem, PC_LOW_ADDR, PC_HIGH_ADDR, value);
}

fn get_sp(mem: &[u8; 2048]) -> u16 {
    read_u16(mem, SP_LOW_ADDR, SP_HIGH_ADDR)
}

fn set_sp(mem: &mut [u8; 2048], value: u16) {
    write_u16(mem, SP_LOW_ADDR, SP_HIGH_ADDR, value);
}

fn get_flags(mem: &[u8; 2048]) -> u8 {
    mem[FLAGS_ADDR as usize]
}

fn set_flags(mem: &mut [u8; 2048], flags: u8) {
    mem[FLAGS_ADDR as usize] = flags;
}

fn update_flags(mem: &mut [u8; 2048], result: u8, carry: bool) {
    let mut flags = get_flags(mem);
    if result == 0 {
        flags |= FLAG_ZERO;
    } else {
        flags &= !FLAG_ZERO;
    }
    if carry {
        flags |= FLAG_CARRY;
    } else {
        flags &= !FLAG_CARRY;
    }
    set_flags(mem, flags);
}

fn get_instruction_info(instruction: u8) -> (&'static str, u8) {
    match instruction {
        0x00 => ("nop", 0),
        0x10 => ("addrm", 3),
        0x11 => ("addmr", 3),
        0x12 => ("addrr", 2),
        0x13 => ("addmm", 4),
        0x20 => ("subrm", 3),
        0x21 => ("submr", 3),
        0x22 => ("subrr", 2),
        0x23 => ("submm", 4),
        0x30 => ("mulrm", 3),
        0x31 => ("mulmr", 3),
        0x32 => ("mulrr", 2),
        0x33 => ("mulmm", 4),
        0x40 => ("divrm", 3),
        0x41 => ("divmr", 3),
        0x42 => ("divrr", 2),
        0x43 => ("divmm", 4),
        0x50 => ("andrm", 3),
        0x51 => ("andmr", 3),
        0x52 => ("andrr", 2),
        0x53 => ("andmm", 4),
        0x60 => ("orrm", 3),
        0x61 => ("ormr", 3),
        0x62 => ("orrr", 2),
        0x63 => ("ormm", 4),
        0x70 => ("ldr", 2),
        0x71 => ("ldm", 3),
        0x72 => ("ldrm", 3),
        0x73 => ("ldmr", 3),
        0x74 => ("ldrr", 2),
        0x75 => ("ldmm", 4),
        0x76 => ("ldri", 2),
        0x77 => ("ldir", 2),
        0x78 => ("ldrio", 3),
        0x79 => ("ldior", 3),
        0x80 => ("push", 1),
        0x90 => ("pop", 1),
        0xA0 => ("jmp", 2),
        0xA1 => ("jz", 2),
        0xA2 => ("jnz", 2),
        0xA3 => ("jc", 2),
        0xA4 => ("jnc", 2),
        0xB0 => ("cmprm", 3),
        0xB1 => ("cmpmr", 3),
        0xB2 => ("cmprr", 2),
        0xB3 => ("cmpmm", 4),
        0xC0 => ("incr", 1),
        0xC1 => ("decr", 1),
        0xC2 => ("incm", 2),
        0xC3 => ("decm", 2),
        0xD0 => ("shlr", 2),
        0xD1 => ("shrr", 2),
        0xD2 => ("shlm", 3),
        0xD3 => ("shrm", 3),
        0xE0 => ("ldrai", 2),
        0xE1 => ("strai", 2),
        0xF0 => ("swprr", 2),
        0xF1 => ("swpmm", 4),
        0xF2 => ("movzr", 2),
        0xF3 => ("movnzr", 2),
        0xF4 => ("movzm", 3),
        0xF5 => ("movnzm", 3),
        0xAA => ("halt", 0),
        _ => ("unknown", 0),
    }
}

struct Emu {
    mem: [u8; 2048],
    halted: bool,
    debug: bool,
    corrupt: bool,
}

impl Emu {
    fn new(rom_path: &str, debug: bool, corrupt: bool) -> Self {
        let mut system = Self {
            mem: [0; 2048],
            halted: false,
            debug,
            corrupt,
        };

        let rom = std::fs::read(rom_path).unwrap_or(FALLBACK_ROM.to_vec());

        let prog_base = ((PROG_BASE_HIGH as u16) << 8) | (PROG_BASE_LOW as u16);
        for (i, byte) in rom.iter().enumerate() {
            if i >= 1264 {
                log::warn!("program too large, skipping remaining bytes");
                break;
            }
            system.mem[(prog_base + i as u16) as usize] = *byte;
        }

        set_pc(&mut system.mem, prog_base);
        let stack_base = ((STACK_BASE_HIGH as u16) << 8) | (STACK_BASE_LOW as u16);
        set_sp(&mut system.mem, stack_base);
        system.mem[FLAGS_ADDR as usize] = 0;
        system.mem[KEY_INPUT_ADDR as usize] = 0;
        system.mem[RANDOM_ADDR as usize] = fastrand::u8(..);

        system
    }

    fn corrupt_mem(&mut self) {
        self.mem[fastrand::usize(0..2048)] = fastrand::u8(..);
    }

    fn step(&mut self) -> bool {
        if self.halted {
            return false;
        }

        let pc = get_pc(&self.mem);
        let instruction = self.mem[pc as usize];

        let (name, arg_count) = get_instruction_info(instruction);
        let mut log_msg = format!("0x{:04X}: {} 0x{:02X}", pc, name, instruction);

        if self.corrupt {
            self.corrupt_mem();
        }

        for i in 1..=arg_count {
            if (pc + i as u16) < 2048 {
                let arg = self.mem[(pc + i as u16) as usize];
                log_msg.push_str(&format!(" 0x{:02X}", arg));
            }
        }
        if self.debug {
            log::debug!("{}", log_msg);
        }

        set_pc(&mut self.mem, pc + 1);

        match instruction {
            0x00 => {}
            0x10 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let (result, carry) = self.mem[addr as usize]
                    .overflowing_add(self.mem[(REG_BASE_ADDR + reg as u16) as usize]);
                self.mem[addr as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0x11 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let (result, carry) = self.mem[(REG_BASE_ADDR + reg as u16) as usize]
                    .overflowing_add(self.mem[addr as usize]);
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0x12 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let (result, carry) = self.mem[(REG_BASE_ADDR + reg1 as u16) as usize]
                    .overflowing_add(self.mem[(REG_BASE_ADDR + reg2 as u16) as usize]);
                self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0x13 => {
                let addr1_low = self.mem[(pc + 1) as usize];
                let addr1_high = self.mem[(pc + 2) as usize];
                let addr2_low = self.mem[(pc + 3) as usize];
                let addr2_high = self.mem[(pc + 4) as usize];
                let addr1 = ((addr1_high as u16) << 8) | (addr1_low as u16);
                let addr2 = ((addr2_high as u16) << 8) | (addr2_low as u16);
                set_pc(&mut self.mem, pc + 5);
                let (result, carry) =
                    self.mem[addr1 as usize].overflowing_add(self.mem[addr2 as usize]);
                self.mem[addr1 as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0x20 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let (result, carry) = self.mem[addr as usize]
                    .overflowing_sub(self.mem[(REG_BASE_ADDR + reg as u16) as usize]);
                self.mem[addr as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0x21 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let (result, carry) = self.mem[(REG_BASE_ADDR + reg as u16) as usize]
                    .overflowing_sub(self.mem[addr as usize]);
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0x22 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let (result, carry) = self.mem[(REG_BASE_ADDR + reg1 as u16) as usize]
                    .overflowing_sub(self.mem[(REG_BASE_ADDR + reg2 as u16) as usize]);
                self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0x23 => {
                let addr1_low = self.mem[(pc + 1) as usize];
                let addr1_high = self.mem[(pc + 2) as usize];
                let addr2_low = self.mem[(pc + 3) as usize];
                let addr2_high = self.mem[(pc + 4) as usize];
                let addr1 = ((addr1_high as u16) << 8) | (addr1_low as u16);
                let addr2 = ((addr2_high as u16) << 8) | (addr2_low as u16);
                set_pc(&mut self.mem, pc + 5);
                let (result, carry) =
                    self.mem[addr1 as usize].overflowing_sub(self.mem[addr2 as usize]);
                self.mem[addr1 as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0x30 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let result = self.mem[(REG_BASE_ADDR + reg as u16) as usize]
                    .wrapping_mul(self.mem[addr as usize]);
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x31 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let result = self.mem[addr as usize]
                    .wrapping_mul(self.mem[(REG_BASE_ADDR + reg as u16) as usize]);
                self.mem[addr as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x32 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let result = self.mem[(REG_BASE_ADDR + reg1 as u16) as usize]
                    .wrapping_mul(self.mem[(REG_BASE_ADDR + reg2 as u16) as usize]);
                self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x33 => {
                let addr1_low = self.mem[(pc + 1) as usize];
                let addr1_high = self.mem[(pc + 2) as usize];
                let addr2_low = self.mem[(pc + 3) as usize];
                let addr2_high = self.mem[(pc + 4) as usize];
                let addr1 = ((addr1_high as u16) << 8) | (addr1_low as u16);
                let addr2 = ((addr2_high as u16) << 8) | (addr2_low as u16);
                set_pc(&mut self.mem, pc + 5);
                let result = self.mem[addr1 as usize].wrapping_mul(self.mem[addr2 as usize]);
                self.mem[addr1 as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x40 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let mem_val = self.mem[addr as usize];
                if mem_val != 0 {
                    let result = self.mem[(REG_BASE_ADDR + reg as u16) as usize] / mem_val;
                    self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                    update_flags(&mut self.mem, result, false);
                }
            }
            0x41 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let reg_val = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                if reg_val != 0 {
                    let result = self.mem[addr as usize] / reg_val;
                    self.mem[addr as usize] = result;
                    update_flags(&mut self.mem, result, false);
                }
            }
            0x42 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let reg2_val = self.mem[(REG_BASE_ADDR + reg2 as u16) as usize];
                if reg2_val != 0 {
                    let result = self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] / reg2_val;
                    self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] = result;
                    update_flags(&mut self.mem, result, false);
                }
            }
            0x43 => {
                let addr1_low = self.mem[(pc + 1) as usize];
                let addr1_high = self.mem[(pc + 2) as usize];
                let addr2_low = self.mem[(pc + 3) as usize];
                let addr2_high = self.mem[(pc + 4) as usize];
                let addr1 = ((addr1_high as u16) << 8) | (addr1_low as u16);
                let addr2 = ((addr2_high as u16) << 8) | (addr2_low as u16);
                set_pc(&mut self.mem, pc + 5);
                let addr2_val = self.mem[addr2 as usize];
                if addr2_val != 0 {
                    let result = self.mem[addr1 as usize] / addr2_val;
                    self.mem[addr1 as usize] = result;
                    update_flags(&mut self.mem, result, false);
                }
            }
            0x50 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let result =
                    self.mem[(REG_BASE_ADDR + reg as u16) as usize] & self.mem[addr as usize];
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x51 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let result =
                    self.mem[addr as usize] & self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                self.mem[addr as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x52 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let result = self.mem[(REG_BASE_ADDR + reg1 as u16) as usize]
                    & self.mem[(REG_BASE_ADDR + reg2 as u16) as usize];
                self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x53 => {
                let addr1_low = self.mem[(pc + 1) as usize];
                let addr1_high = self.mem[(pc + 2) as usize];
                let addr2_low = self.mem[(pc + 3) as usize];
                let addr2_high = self.mem[(pc + 4) as usize];
                let addr1 = ((addr1_high as u16) << 8) | (addr1_low as u16);
                let addr2 = ((addr2_high as u16) << 8) | (addr2_low as u16);
                set_pc(&mut self.mem, pc + 5);
                let result = self.mem[addr1 as usize] & self.mem[addr2 as usize];
                self.mem[addr1 as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x60 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let result =
                    self.mem[(REG_BASE_ADDR + reg as u16) as usize] | self.mem[addr as usize];
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x61 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let result =
                    self.mem[addr as usize] | self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                self.mem[addr as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x62 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let result = self.mem[(REG_BASE_ADDR + reg1 as u16) as usize]
                    | self.mem[(REG_BASE_ADDR + reg2 as u16) as usize];
                self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x63 => {
                let addr1_low = self.mem[(pc + 1) as usize];
                let addr1_high = self.mem[(pc + 2) as usize];
                let addr2_low = self.mem[(pc + 3) as usize];
                let addr2_high = self.mem[(pc + 4) as usize];
                let addr1 = ((addr1_high as u16) << 8) | (addr1_low as u16);
                let addr2 = ((addr2_high as u16) << 8) | (addr2_low as u16);
                set_pc(&mut self.mem, pc + 5);
                let result = self.mem[addr1 as usize] | self.mem[addr2 as usize];
                self.mem[addr1 as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0x70 => {
                let reg = self.mem[(pc + 1) as usize];
                let value = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = value;
            }
            0x71 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                let value = self.mem[(pc + 3) as usize];
                set_pc(&mut self.mem, pc + 4);
                self.mem[addr as usize] = value;
            }
            0x72 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = self.mem[addr as usize];
            }
            0x73 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                let reg = self.mem[(pc + 3) as usize];
                set_pc(&mut self.mem, pc + 4);
                self.mem[addr as usize] = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
            }
            0x74 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] =
                    self.mem[(REG_BASE_ADDR + reg2 as u16) as usize];
            }
            0x75 => {
                let addr1_low = self.mem[(pc + 1) as usize];
                let addr1_high = self.mem[(pc + 2) as usize];
                let addr2_low = self.mem[(pc + 3) as usize];
                let addr2_high = self.mem[(pc + 4) as usize];
                let addr1 = ((addr1_high as u16) << 8) | (addr1_low as u16);
                let addr2 = ((addr2_high as u16) << 8) | (addr2_low as u16);
                set_pc(&mut self.mem, pc + 5);
                self.mem[addr1 as usize] = self.mem[addr2 as usize];
            }
            0x76 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_reg = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let target_addr = self.mem[(REG_BASE_ADDR + addr_reg as u16) as usize] as u16;
                if target_addr < 2048 {
                    self.mem[(REG_BASE_ADDR + reg as u16) as usize] =
                        self.mem[target_addr as usize];
                }
            }
            0x77 => {
                let addr_reg = self.mem[(pc + 1) as usize];
                let reg = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let target_addr = self.mem[(REG_BASE_ADDR + addr_reg as u16) as usize] as u16;
                if target_addr < 2048 {
                    self.mem[target_addr as usize] =
                        self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                }
            }
            0x78 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_reg = self.mem[(pc + 2) as usize];
                let offset = self.mem[(pc + 3) as usize];
                set_pc(&mut self.mem, pc + 4);
                let base_addr = self.mem[(REG_BASE_ADDR + addr_reg as u16) as usize] as u16;
                let target_addr = base_addr.wrapping_add(offset as u16);
                if target_addr < 2048 {
                    self.mem[(REG_BASE_ADDR + reg as u16) as usize] =
                        self.mem[target_addr as usize];
                }
            }
            0x79 => {
                let addr_reg = self.mem[(pc + 1) as usize];
                let offset = self.mem[(pc + 2) as usize];
                let reg = self.mem[(pc + 3) as usize];
                set_pc(&mut self.mem, pc + 4);
                let base_addr = self.mem[(REG_BASE_ADDR + addr_reg as u16) as usize] as u16;
                let target_addr = base_addr.wrapping_add(offset as u16);
                if target_addr < 2048 {
                    self.mem[target_addr as usize] =
                        self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                }
            }
            0x80 => {
                let reg = self.mem[(pc + 1) as usize];
                let value = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                set_pc(&mut self.mem, pc + 2);
                let sp = get_sp(&self.mem);
                set_sp(&mut self.mem, sp + 1);
                self.mem[sp as usize] = value;
            }
            0x90 => {
                let reg = self.mem[(pc + 1) as usize];
                let sp = get_sp(&self.mem);
                if sp > 0 {
                    let new_sp = sp - 1;
                    set_sp(&mut self.mem, new_sp);
                    let value = self.mem[new_sp as usize];
                    self.mem[(REG_BASE_ADDR + reg as u16) as usize] = value;
                }
                set_pc(&mut self.mem, pc + 2);
            }
            0xA0 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, addr);
            }
            0xA1 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                if get_flags(&self.mem) & FLAG_ZERO != 0 {
                    set_pc(&mut self.mem, addr);
                } else {
                    set_pc(&mut self.mem, pc + 3);
                }
            }
            0xA2 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                if get_flags(&self.mem) & FLAG_ZERO == 0 {
                    set_pc(&mut self.mem, addr);
                } else {
                    set_pc(&mut self.mem, pc + 3);
                }
            }
            0xA3 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                if get_flags(&self.mem) & FLAG_CARRY != 0 {
                    set_pc(&mut self.mem, addr);
                } else {
                    set_pc(&mut self.mem, pc + 3);
                }
            }
            0xA4 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                if get_flags(&self.mem) & FLAG_CARRY == 0 {
                    set_pc(&mut self.mem, addr);
                } else {
                    set_pc(&mut self.mem, pc + 3);
                }
            }
            0xB0 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let reg_val = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                let mem_val = self.mem[addr as usize];
                let (result, carry) = reg_val.overflowing_sub(mem_val);
                update_flags(&mut self.mem, result, carry);
            }
            0xB1 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_low = self.mem[(pc + 2) as usize];
                let addr_high = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let mem_val = self.mem[addr as usize];
                let reg_val = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                let (result, carry) = mem_val.overflowing_sub(reg_val);
                update_flags(&mut self.mem, result, carry);
            }
            0xB2 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let reg1_val = self.mem[(REG_BASE_ADDR + reg1 as u16) as usize];
                let reg2_val = self.mem[(REG_BASE_ADDR + reg2 as u16) as usize];
                let (result, carry) = reg1_val.overflowing_sub(reg2_val);
                update_flags(&mut self.mem, result, carry);
            }
            0xB3 => {
                let addr1_low = self.mem[(pc + 1) as usize];
                let addr1_high = self.mem[(pc + 2) as usize];
                let addr2_low = self.mem[(pc + 3) as usize];
                let addr2_high = self.mem[(pc + 4) as usize];
                let addr1 = ((addr1_high as u16) << 8) | (addr1_low as u16);
                let addr2 = ((addr2_high as u16) << 8) | (addr2_low as u16);
                set_pc(&mut self.mem, pc + 5);
                let mem1_val = self.mem[addr1 as usize];
                let mem2_val = self.mem[addr2 as usize];
                let (result, carry) = mem1_val.overflowing_sub(mem2_val);
                update_flags(&mut self.mem, result, carry);
            }
            0xC0 => {
                let reg = self.mem[(pc + 1) as usize];
                set_pc(&mut self.mem, pc + 2);
                let reg_val = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                let (result, carry) = reg_val.overflowing_add(1);
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0xC1 => {
                let reg = self.mem[(pc + 1) as usize];
                set_pc(&mut self.mem, pc + 2);
                let reg_val = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                let (result, carry) = reg_val.overflowing_sub(1);
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0xC2 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 3);
                let mem_val = self.mem[addr as usize];
                let (result, carry) = mem_val.overflowing_add(1);
                self.mem[addr as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0xC3 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 3);
                let mem_val = self.mem[addr as usize];
                let (result, carry) = mem_val.overflowing_sub(1);
                self.mem[addr as usize] = result;
                update_flags(&mut self.mem, result, carry);
            }
            0xD0 => {
                let reg = self.mem[(pc + 1) as usize];
                let amount = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let reg_val = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                let result = reg_val << amount;
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0xD1 => {
                let reg = self.mem[(pc + 1) as usize];
                let amount = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let reg_val = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                let result = reg_val >> amount;
                self.mem[(REG_BASE_ADDR + reg as u16) as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0xD2 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let amount = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let mem_val = self.mem[addr as usize];
                let result = mem_val << amount;
                self.mem[addr as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0xD3 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let amount = self.mem[(pc + 3) as usize];
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                set_pc(&mut self.mem, pc + 4);
                let mem_val = self.mem[addr as usize];
                let result = mem_val >> amount;
                self.mem[addr as usize] = result;
                update_flags(&mut self.mem, result, false);
            }
            0xE0 => {
                let reg = self.mem[(pc + 1) as usize];
                let addr_reg = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let target_addr = self.mem[(REG_BASE_ADDR + addr_reg as u16) as usize] as u16;
                if target_addr < 2048 {
                    self.mem[(REG_BASE_ADDR + reg as u16) as usize] =
                        self.mem[target_addr as usize];
                }
                self.mem[(REG_BASE_ADDR + addr_reg as u16) as usize] += 1;
            }
            0xE1 => {
                let addr_reg = self.mem[(pc + 1) as usize];
                let reg = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let target_addr = self.mem[(REG_BASE_ADDR + addr_reg as u16) as usize] as u16;
                if target_addr < 2048 {
                    self.mem[target_addr as usize] =
                        self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                }
                self.mem[(REG_BASE_ADDR + addr_reg as u16) as usize] += 1;
            }
            0xF0 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                let temp = self.mem[(REG_BASE_ADDR + reg1 as u16) as usize];
                self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] =
                    self.mem[(REG_BASE_ADDR + reg2 as u16) as usize];
                self.mem[(REG_BASE_ADDR + reg2 as u16) as usize] = temp;
            }
            0xF1 => {
                let addr1_low = self.mem[(pc + 1) as usize];
                let addr1_high = self.mem[(pc + 2) as usize];
                let addr2_low = self.mem[(pc + 3) as usize];
                let addr2_high = self.mem[(pc + 4) as usize];
                let addr1 = ((addr1_high as u16) << 8) | (addr1_low as u16);
                let addr2 = ((addr2_high as u16) << 8) | (addr2_low as u16);
                set_pc(&mut self.mem, pc + 5);
                let temp = self.mem[addr1 as usize];
                self.mem[addr1 as usize] = self.mem[addr2 as usize];
                self.mem[addr2 as usize] = temp;
            }
            0xF2 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                if get_flags(&self.mem) & FLAG_ZERO != 0 {
                    self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] =
                        self.mem[(REG_BASE_ADDR + reg2 as u16) as usize];
                }
            }
            0xF3 => {
                let reg1 = self.mem[(pc + 1) as usize];
                let reg2 = self.mem[(pc + 2) as usize];
                set_pc(&mut self.mem, pc + 3);
                if get_flags(&self.mem) & FLAG_ZERO == 0 {
                    self.mem[(REG_BASE_ADDR + reg1 as u16) as usize] =
                        self.mem[(REG_BASE_ADDR + reg2 as u16) as usize];
                }
            }
            0xF4 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let reg = self.mem[(pc + 3) as usize];
                set_pc(&mut self.mem, pc + 4);
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                if get_flags(&self.mem) & FLAG_ZERO != 0 {
                    self.mem[addr as usize] = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                }
            }
            0xF5 => {
                let addr_low = self.mem[(pc + 1) as usize];
                let addr_high = self.mem[(pc + 2) as usize];
                let reg = self.mem[(pc + 3) as usize];
                set_pc(&mut self.mem, pc + 4);
                let addr = ((addr_high as u16) << 8) | (addr_low as u16);
                if get_flags(&self.mem) & FLAG_ZERO == 0 {
                    self.mem[addr as usize] = self.mem[(REG_BASE_ADDR + reg as u16) as usize];
                }
            }
            0xAA => {
                self.halted = true;
                return false;
            }
            _ => {
                log::error!("unknown instruction: {} at pc: {}", instruction, pc);
                self.halted = true;
                return false;
            }
        }
        true
    }

    fn get_video_memory(&self) -> &[u8] {
        &self.mem[0..256]
    }

    fn update_key_input(&mut self, key_state: u8) {
        self.mem[KEY_INPUT_ADDR as usize] = key_state;
    }

    fn get_debug_info(&self) -> (u16, u8, u8) {
        let pc = get_pc(&self.mem);
        let flags = get_flags(&self.mem);
        let current_instruction = if pc < 2048 { self.mem[pc as usize] } else { 0 };
        (pc, flags, current_instruction)
    }

    fn update_random(&mut self) {
        self.mem[RANDOM_ADDR as usize] = fastrand::u8(..);
    }
}

fn get_color_palette(index: u8) -> [u8; 4] {
    match index {
        0 => [0x00, 0x00, 0x00, 0xff],  // black
        1 => [0xff, 0xff, 0xff, 0xff],  // white
        2 => [0xff, 0x00, 0x00, 0xff],  // red
        3 => [0x00, 0xff, 0x00, 0xff],  // green
        4 => [0x00, 0x00, 0xff, 0xff],  // blue
        5 => [0xff, 0xff, 0x00, 0xff],  // yellow
        6 => [0xff, 0x00, 0xff, 0xff],  // magenta
        7 => [0x00, 0xff, 0xff, 0xff],  // cyan
        8 => [0x80, 0x80, 0x80, 0xff],  // gray
        9 => [0x80, 0x00, 0x00, 0xff],  // dark red
        10 => [0x00, 0x80, 0x00, 0xff], // dark green
        11 => [0x00, 0x00, 0x80, 0xff], // dark blue
        12 => [0x80, 0x80, 0x00, 0xff], // dark yellow
        13 => [0x80, 0x00, 0x80, 0xff], // dark magenta
        14 => [0x00, 0x80, 0x80, 0xff], // dark cyan
        15 => [0xc0, 0xc0, 0xc0, 0xff], // light gray
        _ => [0x40, 0x40, 0x40, 0xff],  // default gray
    }
}

fn main() -> Result<(), Error> {
    env_logger::Builder::from_default_env()
        .filter_level(log::LevelFilter::Off)
        .filter_module("gamekid", log::LevelFilter::Debug)
        .init();

    let args = Args::parse();

    let event_loop = EventLoop::new().unwrap();
    let mut input = WinitInputHelper::new();
    let window = {
        let size = LogicalSize::new(WINDOW_WIDTH as f64, WINDOW_HEIGHT as f64);
        WindowBuilder::new()
            .with_title("GameKid")
            .with_inner_size(size)
            .with_min_inner_size(size)
            .with_max_inner_size(size)
            .with_resizable(false)
            .with_enabled_buttons(WindowButtons::CLOSE | WindowButtons::MINIMIZE)
            .build(&event_loop)
            .unwrap()
    };

    let mut pixels = {
        let window_size = window.inner_size();
        let surface_texture = SurfaceTexture::new(window_size.width, window_size.height, &window);
        Pixels::new(SCREEN_WIDTH, SCREEN_HEIGHT, surface_texture)?
    };

    let mut system = Emu::new(args.rom.as_deref().unwrap_or(""), args.debug, args.corrupt);
    let mut cycles_per_frame = 0;
    let mut key_state: u8 = 0;

    let res = event_loop.run(|event, elwt| {
        if let Event::WindowEvent {
            event: WindowEvent::RedrawRequested,
            ..
        } = event
        {
            for _ in 0..cycles_per_frame {
                system.update_random();
                if !system.step() {
                    break;
                }
            }

            draw_screen(&system, pixels.frame_mut(), cycles_per_frame);
            if let Err(err) = pixels.render() {
                log::error!("pixels.render() failed: {err}");
                elwt.exit();
                return;
            }
        }

        if input.update(&event) {
            if input.key_pressed(KeyCode::Escape) || input.close_requested() {
                elwt.exit();
                return;
            }

            if input.key_pressed(KeyCode::Space) {
                cycles_per_frame = if cycles_per_frame == 0 { 5 } else { 0 };
            }

            if input.key_pressed(KeyCode::Tab) {
                system.update_random();
                system.step();
            }

            let mut new_key_state = 0;
            if input.key_held(KeyCode::ArrowUp) {
                new_key_state |= KEY_UP;
            }
            if input.key_held(KeyCode::ArrowDown) {
                new_key_state |= KEY_DOWN;
            }
            if input.key_held(KeyCode::ArrowLeft) {
                new_key_state |= KEY_LEFT;
            }
            if input.key_held(KeyCode::ArrowRight) {
                new_key_state |= KEY_RIGHT;
            }
            if input.key_held(KeyCode::KeyZ) {
                new_key_state |= KEY_A;
            }
            if input.key_held(KeyCode::KeyX) {
                new_key_state |= KEY_B;
            }

            if new_key_state != key_state {
                key_state = new_key_state;
                system.update_key_input(key_state);
            }

            window.request_redraw();
        }
    });
    res.map_err(|e| Error::UserDefined(Box::new(e)))
}

fn draw_screen(system: &Emu, frame: &mut [u8], cycles_per_frame: u32) {
    let video_mem = system.get_video_memory();
    let (pc, flags, current_inst) = system.get_debug_info();

    for (i, pixel) in frame.chunks_exact_mut(4).enumerate() {
        let x = i % SCREEN_WIDTH as usize;
        let y = i / SCREEN_WIDTH as usize;

        if y == 0 {
            let color_index = draw_debug_pixel(
                x,
                pc,
                flags,
                current_inst,
                &system.mem,
                cycles_per_frame > 0,
            );
            let rgba = get_color_palette(color_index);
            pixel.copy_from_slice(&rgba);
        } else {
            let video_y = y - 1;
            let video_index = video_y * SCREEN_WIDTH as usize + x;
            let color_index = video_mem[video_index];
            let rgba = get_color_palette(color_index);
            pixel.copy_from_slice(&rgba);
        }
    }
}

fn draw_debug_pixel(
    x: usize,
    pc: u16,
    flags: u8,
    current_inst: u8,
    mem: &[u8; 2048],
    is_running: bool,
) -> u8 {
    match x {
        0..=1 => ((pc >> (8 * (1 - x))) & 0xFF) as u8 >> 4,
        2 => flags >> 4,
        3 => current_inst >> 4,
        4..=14 => {
            let reg_index = x - 4;
            if reg_index < 20 {
                let reg_addr = REG_BASE_ADDR as usize + reg_index;
                mem.get(reg_addr).unwrap_or(&0) >> 4
            } else {
                0
            }
        }
        15 => {
            if is_running {
                2
            } else {
                0
            }
        }
        _ => 0,
    }
}