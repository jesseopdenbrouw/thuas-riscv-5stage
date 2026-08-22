-- #################################################################################################
-- # core.vhd - The 5-stage processor core                                                         #
-- # ********************************************************************************************* #
-- # This file is part of the THUAS RISCV RV32 Project                                             #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2026, Jesse op den Brouw. All rights reserved.                                  #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # https:/github.com/jesseopdenbrouw/thuas-riscv                                                 #
-- #################################################################################################

-- This file contains the description of a RISC-V RV32IM core,
-- implemented with a 5-stage classic pipeline, plus a register
-- file bypass, if the register file is not write-through.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
-- synthesis translate_off
use ieee.std_logic_textio.all;
use std.textio.all;
-- synthesis translate_on

library work;
use work.processor_common.all;

entity core is
    generic (
          -- The frequency of the system
          SYSTEM_FREQUENCY : integer;
          -- Hardware version in BCD
          HW_VERSION : integer;
          -- Do we have on-chip debugger (OCD)?
          HAVE_OCD : boolean;
          -- If bootloader enabled, adjust the boot address
          HAVE_BOOTLOADER_ROM : boolean;
          -- Disable CSR address check when in debug mode
          OCD_CSR_CHECK_DISABLE : boolean;
          -- RISCV E (embedded) or RISCV I (full)
          HAVE_RISCV_E : boolean;
          -- Do we have the integer multiply/divide unit?
          HAVE_MULDIV : boolean;
          -- Fast divide (needs more area)?
          FAST_DIVIDE : boolean;
          -- Do we have Zba (sh?add)
          HAVE_ZBA : boolean;
          -- Do we have Zbb (bit instructions)?
          HAVE_ZBB : boolean;
          -- Do we have Zbs (bit instructions)?
          HAVE_ZBS : boolean;
          -- Do we have Zicond (czero.{eqz|nez})?
          HAVE_ZICOND : boolean;
          -- Have Zimop?
          HAVE_ZIMOP : boolean;
          -- Have Zbkb (bitmanip instructions for cryptography)
          HAVE_ZBKB : boolean;
          -- Do we have HPM counters?
          HAVE_ZIHPM : boolean;
          -- Do we enable vectored mode for mtvec?
          VECTORED_MTVEC : boolean;
          -- Do we have registers is RAM?
          HAVE_REGISTERS_IN_RAM : boolean;
          -- 4 high bits of ROM address
          ROM_HIGH_NIBBLE : memory_high_nibble;
          -- 4 high bits of boot ROM address
          BOOT_HIGH_NIBBLE : memory_high_nibble;
          -- Buffer I/O response
          BUFFER_IO_RESPONSE : boolean;
          -- Fast memory access (severly reduces Fmax)?
          FAST_MEM : boolean;
          -- Do we have UART1?
          HAVE_UART1 : boolean;
          -- Do we have UART2?
          HAVE_UART2 : boolean;
          -- Do we have SPI1?
          HAVE_SPI1 : boolean;
          -- Do we have SPI2?
          HAVE_SPI2 : boolean;
          -- Do we have I2C1?
          HAVE_I2C1 : boolean;
          -- Do we have I2C2?
          HAVE_I2C2 : boolean;
          -- Do we have TIMER1?
          HAVE_TIMER1 : boolean;
          -- Do we have TIMER2?
          HAVE_TIMER2 : boolean;
          -- Use Machine-mode Software Interrupt?
          HAVE_MSI : boolean;
          -- Use watchdog?
          HAVE_WDT : boolean;
          -- Use CRC?
          HAVE_CRC : boolean;
          -- UART1 BREAK triggers system reset
          UART1_BREAK_RESETS : boolean
         );
    port (I_clk : in std_logic;
          I_areset : in std_logic;
          I_sreset : in std_logic;
          -- Instruction request from ROM
          O_instr_request : out instr_request_type;
          I_instr_response : in instr_response_type;
          -- To and from memory
          O_bus_request : out bus_request_type;
          I_bus_response : in bus_response_type;
          -- Interrupt signals from I/O
          I_intrio : data_type;
          -- [m]time from the memory mapped I/O
          I_mtime : in data_type;
          I_mtimeh : in data_type;
          -- Debug signals
          I_dm_core_data_request : in dm_core_data_request_type;
          O_dm_core_data_response : out dm_core_data_response_type;
          I_halt_req : in std_logic;
          I_resume_req : in std_logic;
          I_ackhavereset : in std_logic;
          O_halt_ack : out std_logic;
          O_reset_ack : out std_logic;
          O_resume_ack : out std_logic
         );
end entity core;

architecture rtl of core is

-- Number of registers: 16 for E, 32 for I
constant NUMBER_OF_REGISTERS : integer := get_int_from_boolean(HAVE_RISCV_E, 16, 32);

-- Do we want the extra simulation output file?
constant SIMULATION_EXTRA : boolean := false;

-- The Program Counter
-- Not part of any record.
-- Make sure to start with the correct PC direct after download to target
signal pc : data_type := get_std_logic_vector_from_boolean(HAVE_BOOTLOADER_ROM, BOOT_HIGH_NIBBLE, ROM_HIGH_NIBBLE) & x"0000000";

-- IF/ID signals for Instruction Decode stage
type if_id_type is record
    pc : data_type;
    selrs1 : integer range 0 to NUMBER_OF_REGISTERS-1;
    selrs2 : integer range 0 to NUMBER_OF_REGISTERS-1;
end record if_id_type;
signal if_id : if_id_type;


-- ID/EX signals for Execute stage
-- Behavior of the Program Counter
type pc_op_type is (pc_incr, pc_hold, pc_loadoffset, pc_loadoffsetregister,
                    pc_branch, pc_load_mepc, pc_load_mtvec);
type id_ex_type is record
    -- Generic signals
    instr : data_type;
    alu_op : alu_op_type;
    rs1data : data_type;
    rs2data : data_type;
    rs1 : reg_type;
    rs2 : reg_type;
    rd : reg_type;
    rd_en : std_logic;
    -- PC operation on
    pc_op : pc_op_type;
    pc : data_type;
    -- Immediate & unsigned
    imm : data_type;
    isimm : std_logic;
    isunsigned : std_logic;
    -- Multiply/divide operation
    md_op : func3_type;
    md_start : std_logic;
    -- Memory operation
    isload : std_logic;
    isstore : std_logic;
    memaccess : memaccess_type;
    memsize : memsize_type;    
    -- CSR operation
    csr_op : csr_op_type;
    csr_addr : std_logic_vector(11 downto 0);
    csr_immrs1 : std_logic_vector(4 downto 0);
    -- Instruction execute valid
    valid : std_logic;
    -- Test signals can be removed
    a, b, c, r : data_type;
end record id_ex_type;
signal id_ex : id_ex_type;

-- EX/MEM for memory stage
type ex_mem_type is record
    -- needed for wb stage
    alu_op : alu_op_type;
    rs1data : data_type;
    rs2data : data_type;
    rs1 : reg_type;
    rs2 : reg_type;
    rd : reg_type;
    rd_en : std_logic;
    memaccess : memaccess_type;
    memsize : memsize_type;
    isload : std_logic;
    isstore : std_logic;
    pc : data_type;
    -- Instruction execute valid
    valid : std_logic;
end record ex_mem_type;
signal ex_mem : ex_mem_type;


--MEM/WB stage
type mem_wb_type is record
    alu_op : alu_op_type;
    rd : reg_type;
    rddata : data_type;
    rd_en : std_logic;
    isload : std_logic;
    rs1data : data_type;
    -- Instruction execute valid
    valid : std_logic;
end record mem_wb_type;
signal mem_wb : mem_wb_type;

-- WB/BY stage (register file bypass)
type wb_bp_type is record
    rd : reg_type;
    rd_en : std_logic;
    rddata : data_type;
    -- Instruction execute valid
    valid : std_logic;
end record wb_bp_type;
signal wb_bp : wb_bp_type;

-- Multiplier/divider
type md_type is record
    -- Operation ready
    ready : std_logic;
    -- Multiplier
    rdata_a, rdata_b : unsigned(32 downto 0);
    mul_rd_int : signed(65 downto 0);
    mul_running : std_logic;
    mul_ready : std_logic;
    mul : data_type;
    -- Divider
    buf : unsigned(63 downto 0);
    divisor : unsigned(31 downto 0);
    quotient : unsigned(31 downto 0);
    remainder : unsigned(31 downto 0);
    outsign : std_logic;
    div_ready : std_logic;
    div : data_type;
-- synthesis translate_off
    count: integer range 0 to 32;
-- synthesis translate_on
end record md_type;
signal md : md_type;
alias md_buf1 is md.buf(63 downto 32);
alias md_buf2 is md.buf(31 downto 0);

-- The registers
type regs_array_type is array (0 to NUMBER_OF_REGISTERS-1) of data_type;
-- After reconfiguration, all the registers are loaded with null-bits
constant reg_contents : regs_array_type := (  (others => (others => '0')) );
-- Quartus will not generate RAM blocks for registers when they are in a record
signal regs_rs1 : regs_array_type := reg_contents;
signal regs_rs2 : regs_array_type := reg_contents;
-- Special for debugging
signal regs_dbg : regs_array_type := reg_contents;
-- Used with on-chip debugger
signal data_from_gpr : data_type;

-- Control signals
-- States of the controller
type state_type is (state_boot0, state_boot1, state_exec,
                    state_flush, state_flush2, state_md, state_md2,
                    state_trap, state_trap2, state_trap3, state_mret,
                    state_mret2, state_wfi, state_debugpre1,
                    state_debugpre2, state_debug, state_debugflush,
                    state_debugflush2, state_debugflush3);
type control_type is record
    -- Stall, flush, jump/branch, instructions retired
    stall : std_logic;
    flush : std_logic;
    penalty : std_logic;
    instret : std_logic;
    -- The state of the controller
    state : state_type;
    -- Forwarding
    forwarda : std_logic_vector(1 downto 0);
    forwardb : std_logic_vector(1 downto 0);
    -- Do we have a use-after-load hazard?
    loadhazard : std_logic;
    -- Instruction problem
    illegal_instruction_decode : std_logic;
    illegal_instruction_csr : std_logic;
    instruction_misaligned : std_logic;
    instr_access_error : std_logic_vector(1 downto 0);
    instr_misaligned_ff : std_logic;
    -- Instructions related to traps
    ecall_request : std_logic;
    ebreak_request : std_logic;
    wfi_request : std_logic;
    mret_request : std_logic;
    -- Trap related
    trap_request : std_logic;
    trap_release : std_logic;
    trap_memfault : std_logic;
    trap_mcause : data_type;
    may_interrupt : std_logic;
    nmi_lockout : std_logic;
    mret_request_delay : std_logic;
    -- Debug related
    stall_on_debug : std_logic;
    indebug : std_logic;
    load_pc : std_logic;
    load_dpc : std_logic;
    skip_match : std_logic;
    bpmatch : std_logic;
    step : std_logic;
    isstepping : std_logic;
end record control_type;
signal control : control_type;

-- The Control and Status Registers
-- Keep csr_size_bits to 12!!!
-- The CSRs have their own address space, it is not
-- visible on the 4 GB normal address space.
constant csr_size_bits : integer := 12;
constant csr_size : integer := 2**csr_size_bits;

-- CSR registers
type csr_reg_type is record
    -- Standard M-mode
    mvendorid : data_type;
    marchid : data_type;
    mimpid : data_type;
    mhartid : data_type;
    misa : data_type;
    mstatus : data_type;
    -- Basic counters
    mcycle : data_type;
    mcycleh : data_type;
    mtime : data_type;
    mtimeh : data_type;
    minstret : data_type;
    minstreth : data_type;
    mcountinhibit : data_type;
    -- Trap related
    mtvec : data_type;
    mepc : data_type;
    mscratch : data_type;
    mtval : data_type;
    mie : data_type;
    mip : data_type;
    mcause : data_type;
    -- Custom read-ony
    mxhw : data_type;
    mxspeed : data_type;
    -- Debug related
    dcsr : data_type;
    dpc : data_type;
    tselect : data_type;
    tdata1 : data_type;
    tdata2 : data_type;
    tinfo : data_type;
    dcsr_cause : std_logic_vector(3 downto 0);    
end record csr_reg_type;
signal csr_reg : csr_reg_type;

-- Signals to and from CSR
type csr_access_type is record
    op : csr_op_type;
    address : std_logic_vector(11 downto 0);
    immrs1 : std_logic_vector(4 downto 0);
    data_from_csr : data_type;
    data_to_csr : data_type;
end record csr_access_type;
signal csr_access : csr_access_type;

-- For transfering data between CSR and the PC
type csr_transfer_type is record
    mtvec_to_pc : data_type;
    mepc_to_pc : data_type;
    address_to_mtval : data_type;
    dpc_to_pc : data_type;
end record csr_transfer_type;
signal csr_transfer : csr_transfer_type;


begin

    --
    -- Control block
    --
    -- This block holds the current processing state of the
    -- processor and supplies the control signals to the
    -- other blocks.
    --

    -- Hardware breakpoint match
    control.bpmatch <= '1' when id_ex.pc = csr_reg.tdata2 and                   -- instruction address match
                                csr_reg.tdata1(6) = '1' and                     -- and M mode
                                csr_reg.tdata1(2) = '1' and                     -- and EXECUTE
                                csr_reg.tdata1(15 downto 12) = "0001" and       -- and enter debug mode
                                csr_reg.tdata1(10 downto 07) = "0000" else      -- and match equal   
                       '0'; 
    
    process (I_clk, I_areset) is
    begin
        if I_areset = '1' then
            control.state <= state_boot0;
            control.step <= '0';
            O_halt_ack <= '0';
            O_reset_ack <= '0';
            O_resume_ack <= '0';
            control.load_pc <= '0';
            control.load_dpc <= '0';
            control.skip_match <= '0';
            csr_reg.dcsr_cause <= "0000";
        elsif rising_edge(I_clk) then
            if I_sreset = '1' then
                control.state <= state_boot0;
                control.step <= '0';
                O_halt_ack <= '0';
                O_reset_ack <= '0';
                O_resume_ack <= '0';
                control.load_pc <= '0';
                control.load_dpc <= '0';
                control.skip_match <= '0';
                csr_reg.dcsr_cause <= "0000";
            else
                control.load_pc <= '0';
                control.load_dpc <= '0';
                if I_ackhavereset = '1' then
                    O_reset_ack <= '0';
                end if;
            
                case control.state is
                    -- First cycle after reset, flushes pipeline
                    when state_boot0 =>
                        control.state <= state_boot1;
                        control.skip_match <= '0';
                        control.step <= '0';
                        O_halt_ack <= '0';
                        O_reset_ack <= '0';
                        O_resume_ack <= '0';
                        csr_reg.dcsr_cause <= "0000";
                    -- Second cycle after reset
                    when state_boot1 =>
                        control.state <= state_exec;
                        O_reset_ack <= '1';
                    -- Instruction execute
                    when state_exec =>
                        O_resume_ack <= '1';  -- keep this at '1'
                        O_halt_ack <= '0';
                        
                        -- If user halt request...
                        if I_halt_req = '1' and HAVE_OCD then
--                            control.step <= '0';                --
                            control.state <= state_debugpre1;   -- Goto to debug state
                            control.load_dpc <= '1';            -- Load DPC with PC
                            csr_reg.dcsr_cause <= "1011";       -- Signal halt to user
                        -- If hardware breakpoint and not resuming from this breakpoint...
                        elsif control.bpmatch = '1' and control.skip_match = '0' and HAVE_OCD then
--                            control.step <= '0';                --
                            control.state <= state_debugpre1;   -- Goto debug state
                            control.load_dpc <= '1';            -- Load DPC with PC
                            csr_reg.dcsr_cause <= "1010";       -- Signal HW break to user
                        -- If software breakpoint...
                        -- Can be switched off with: <targetname> riscv set_ebreakm off
                        -- dcsr(15) is dcsr.ebreakm bit
                        elsif control.ebreak_request = '1' and csr_reg.dcsr(15) = '1' and HAVE_OCD then
--                            control.step <= '0';                --
                            control.state <= state_debugpre1;   -- Goto debug state
                            control.load_dpc <= '1';            -- Load DPC with PC
                            csr_reg.dcsr_cause <= "1001";       -- Signal EBREAK to user
                        -- If we are stapping and not resuming from this breakpoint...
                        elsif control.isstepping = '1' and control.step = '0' and HAVE_OCD then
--                            control.step <= '0';
                            control.state <= state_debugpre1;   -- Goto debug state
                            control.load_dpc <= '1';            -- Load DPC with PC
                            csr_reg.dcsr_cause <= "1100";       -- Signal STEP to user

                        -- Do we have a trap?
                        elsif control.trap_request = '1' then
                            control.state <= state_trap;
                        -- If a WFI instruction is run
                        elsif control.wfi_request = '1' then
                            control.state <= state_wfi;
                        -- If we have an mret request (MRET)
                        elsif control.mret_request = '1' then
                            control.state <= state_mret;
                        -- Do we have to jump
                        elsif control.penalty = '1' then
                            control.state <= state_flush;
                        -- Do we start an MD cycle?
                        elsif id_ex.md_start = '1' then
                            control.state <= state_md;
                        else
                            control.state <= state_exec;
                        end if;
                        control.step <= '0';
                    -- Flush IF and ID
                    when state_flush =>
                        control.state <= state_flush2;
                    -- Flush IF and ID
                    when state_flush2 =>
                        control.state <= state_exec;
                    -- MD operation in progress (cannot be interrupted)
                    when state_md =>
                        if md.ready = '1' then
                            control.state <= state_md2;
                        end if;
                    -- MD ready, copy result (cannot be interrupted)
                    when state_md2 =>
                        control.state <= state_exec;
                    -- Starting a trap
                    when state_trap =>
                        control.state <= state_trap2;
                    when state_trap2 =>
                        if control.isstepping = '1' then
                            control.state <= state_trap3;
                        else
                            control.state <= state_exec;
                        end if;
                    -- Need this extra state to load the correct PC in DPC
                    when state_trap3 =>
                        control.state <= state_exec;
                    -- First state of MRET, flushes the pipeline
                    when state_mret =>
                        control.state <= state_mret2;
                    -- Second state of MRET, flushes the pipeline
                    when state_mret2 =>
                        control.state <= state_exec;
                    -- Wait for an interrupt (cannot be an exception)
                    when state_wfi =>
                        -- A halt request on WFI goes to debug state
                        if I_halt_req = '1' and HAVE_OCD then
                            control.step <= '0';                --
                            control.state <= state_debugpre1;   -- Goto to debug state
                            control.load_dpc <= '1';            -- Load DPC with PC
                            csr_reg.dcsr_cause <= "1011";       -- Signal halt to user                  
                        elsif control.trap_request = '1' then
                            control.state <= state_trap;
                        end if;
                    -- Start entering debug mode, needed to get latest
                    -- data in register file before entering debug
                    when state_debugpre1 =>
                        control.state <= state_debugpre2;
                    when state_debugpre2 =>
                        O_halt_ack <= '1';
                        control.state <= state_debug;
                    -- When we're in debug...
                    when state_debug =>
                        csr_reg.dcsr_cause <= "0000";
                        -- If resuming from stepping and we are stepping...
                        if I_resume_req = '1' and control.isstepping = '1' then
                            control.state <= state_debugflush;   -- Flush the pipeline
                            control.step <= '1';                 -- Set stepping
                            csr_reg.dcsr_cause <= "1000";        -- Clear DCSR.cause
                            control.load_pc <= '1';              -- Load PC with DPC
                            O_halt_ack <= '0';                   -- Signal run
                            O_resume_ack <= '0';                 -- Temporary low
                            control.skip_match <= control.bpmatch;
                        -- If we are resuming and not stepping...
                        elsif I_resume_req = '1' then
                            control.state <= state_debugflush;   -- Flush the pipeline
                            control.step <= '0';                 -- Not stepping
                            csr_reg.dcsr_cause <= "1000";        -- Clear DCSR.cause
                            control.load_pc <= '1';              -- Load PC with DPC
                            O_halt_ack <= '0';                   -- Signal run
                            O_resume_ack <= '0';                 -- Temporary low
                            control.skip_match <= control.bpmatch;
                        end if;
                    -- Flush after leaving debug state
                    when state_debugflush =>
                        control.state <= state_debugflush2;
                    when state_debugflush2 =>
                        control.state <= state_debugflush3;
                    when state_debugflush3 =>
                        control.state <= state_exec;
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process;
    
    -- We need to stall (on use-after-load, MD operation or WFI)
    control.stall <= '1' when control.loadhazard = '1' or 
                             (control.state = state_exec and id_ex.md_start = '1') or
                              control.state = state_md or
                              control.state = state_wfi
                         else '0';
                         
    -- If we got a trigger (breakpoint (hw/sw) or step)
    control.stall_on_debug <= '1' when (control.state = state_exec and I_halt_req = '1' and HAVE_OCD) or
                                       (control.state = state_exec and control.bpmatch = '1' and control.skip_match = '0' and HAVE_OCD) or
                                       (control.state = state_exec and control.ebreak_request = '1' and csr_reg.dcsr(15) = '1' and HAVE_OCD) or
                                       (control.state = state_debugpre1 and HAVE_OCD) or
                                       (control.state = state_debugpre2 and HAVE_OCD) or
                                       (control.state = state_debug and HAVE_OCD) or
                                       (control.state = state_exec and control.isstepping = '1' and control.step = '0' and HAVE_OCD)
                                  else '0';
                                  
    -- In debug mode
    control.indebug <= '1' when control.state = state_debug and HAVE_OCD else '0';


    -- Stall the ROM
    O_instr_request.stall <= control.stall or control.stall_on_debug;
    
    -- We're stepping
    control.isstepping <= '1' when csr_reg.dcsr(2) = '1' else '0';
    
    -- We're flushing the pipeline
    control.flush <= '1' when control.penalty = '1' or          -- if branching
                              control.wfi_request = '1' or      -- if WFI
                              control.loadhazard = '1' or       -- if load hazard    <-- extra info needed
                              control.state = state_flush or    -- do to branching
                              control.state = state_debugflush or  -- leaving debug
                              control.state = state_debugflush2 or
                              control.state = state_trap or     -- when entering trap
                              control.state = state_trap2 or    -- when entering trap
                              control.state = state_mret or     -- if returning from IH    
                              control.state = state_boot0       -- when after reset
                         else '0';

    -- Delay the release request (for use in the CSR)
    control.mret_request_delay <= '1' when control.state = state_mret2 else '0';

    -- Update the instret CSR counter
    control.instret <= mem_wb.valid;

    -- May the core be interrupted (only for interrupts, not exceptions), but not during stepping?
    control.may_interrupt <= '1' when (control.state = state_exec or control.state = state_wfi) and control.isstepping = '0' else '0';
    
    -- Check if the currently executing instruction address is aligned to word
    -- We need to make a one-shot, because the PC is incremented by 4 each clock
    -- cycle so the pipeline PCs will all be misaligned and the misaligned signal
    -- would be multiple clock cycles which messes up trap entry.
    process (I_clk, I_areset) is
    begin
        if I_areset = '1' then
            control.instr_misaligned_ff <= '0';
        elsif rising_edge(I_clk) then
            if I_sreset = '1' then
                control.instr_misaligned_ff <= '0';
            elsif id_ex.pc(1 downto 0) /= "00" then
                control.instr_misaligned_ff <= '1';
            else
                control.instr_misaligned_ff <= '0';
            end if;
        end if;
    end process;
    control.instruction_misaligned <= '1' when id_ex.pc(1 downto 0) /= "00" and control.instr_misaligned_ff = '0' else '0';
            
    -- Delay instruction access (read) error for two cycles so that
    -- the faulted address/instruction is being processed in the
    -- execute stage.
    process (I_clk, I_areset) is
    begin
        if I_areset = '1' then
            control.instr_access_error <= (others => '0');
        elsif rising_edge(I_clk) then
            if I_sreset = '1' then
                control.instr_access_error <= (others => '0');
            elsif control.state = state_trap then
                control.instr_access_error <= (others => '0');
            elsif control.instr_access_error(1) = '1' then
                control.instr_access_error <= (others => '0');
            else
                control.instr_access_error <= control.instr_access_error(0) & I_instr_response.instr_access_error;
            end if;
        end if;
    end process;

    
    --
    -- Instruction fetch block
    -- This block controls the instruction fetch from the ROM.
    -- It also instructs the PC to load a new address, either
    -- the next sequential address or a jump target address.
    --
    
    -- The PC
    process (I_clk, I_areset) is
    variable a_v : data_type;
    begin
        -- Asynchronous reset
        if I_areset = '1' then
            pc <= (others => '0');
            if HAVE_BOOTLOADER_ROM then
                pc(pc'left downto pc'left-3) <= BOOT_HIGH_NIBBLE;
            else
                pc(pc'left downto pc'left-3) <= ROM_HIGH_NIBBLE;
            end if;
        elsif rising_edge(I_clk) then
            -- Synchronous reset
            if I_sreset = '1' then
                pc <= (others => '0');
                if HAVE_BOOTLOADER_ROM then
                    pc(pc'left downto pc'left-3) <= BOOT_HIGH_NIBBLE;
                else
                    pc(pc'left downto pc'left-3) <= ROM_HIGH_NIBBLE;
                end if;
            else
                -- Load DPC to PC
                if control.load_pc = '1' then
                    pc <= csr_transfer.dpc_to_pc;
                -- Should we stall the pipeline
                elsif control.stall = '1' or control.stall_on_debug = '1' then
                    -- PC holds value
                    null;
                else
                    case id_ex.pc_op is
                        -- Hold the PC
                        when pc_hold =>
                            null;
                        -- Increment the PC
                        when pc_incr =>
                            pc <= std_logic_vector(unsigned(pc) + 4);
                        -- JAL
                        when pc_loadoffset =>
                            pc <= std_logic_vector(unsigned(id_ex.pc) + unsigned(id_ex.imm));
                        -- JALR
                        when pc_loadoffsetregister =>
                            if control.forwarda = "10" then
                                a_v := ex_mem.rs1data;
                            elsif control.forwarda = "01" then
                                a_v := mem_wb.rddata;
                            elsif control.forwarda = "11" then
                                a_v := wb_bp.rddata;
                            else
                                a_v := id_ex.rs1data;
                            end if;
                            pc <= std_logic_vector(unsigned(a_v) + unsigned(id_ex.imm));
--                            if control.forwarda = "10" then
--                                pc <= std_logic_vector(unsigned(ex_mem.rs1data) + unsigned(id_ex.imm));
--                            elsif control.forwarda = "01" then
--                                pc <= std_logic_vector(unsigned(mem_wb.rddata) + unsigned(id_ex.imm));
--                            elsif control.forwarda = "11" then
--                                pc <= std_logic_vector(unsigned(wb_bp.rddata) + unsigned(id_ex.imm));
--                            else
--                                pc <= std_logic_vector(unsigned(id_ex.rs1data) + unsigned(id_ex.imm));
--                            end if;
                            -- As per RISC-V unpriv spec (1.1.5.1. Unconditional Jumps)
                            pc(0) <= '0';
                        -- Branch
                        when pc_branch =>
                            -- Must we branch?
                            if control.penalty = '1' then
                                pc <= std_logic_vector(unsigned(id_ex.pc) + unsigned(id_ex.imm));
                            else
                                pc <= std_logic_vector(unsigned(pc) + 4);
                            end if;
                        -- Load mtvec, direct or vectored
                        when pc_load_mtvec =>
                            pc <= csr_transfer.mtvec_to_pc;
                        -- Load mepc
                        when pc_load_mepc =>
                            pc <= csr_transfer.mepc_to_pc;
                        when others =>
                            pc <= std_logic_vector(unsigned(pc) + 4);
                    end case;
                end if;
            end if; -- sreset
        end if; -- posedge
    end process;
    -- For fetching instructions
    O_instr_request.pc <= pc;

    
    -- The PC at the fetched instruction
    process (I_clk, I_areset) is
    variable instr_var : data_type;
    begin
        if I_areset = '1' then
            -- Set at 0x00000000 because after reset
            -- the processor will run for two booting
            -- states. After that, this PC will follow
            -- the PC.
            if_id.pc <= (others => '0');
        elsif rising_edge(I_clk) then
            if I_sreset = '1' then
                if_id.pc <= (others => '0');
            -- Must we stall?
            elsif control.stall = '1' or control.stall_on_debug = '1' or id_ex.pc_op = pc_hold then
                null;
            else
                if_id.pc <= pc;
            end if;
        end if;
    end process;
    
    
    -- Forwarding: check if we need forwarding data
    -- Extra stage to bypass register file (WB/BP)
    process (id_ex, ex_mem, mem_wb, wb_bp) is
    begin
        if id_ex.rs1 = ex_mem.rd and ex_mem.rd_en = '1' then
            control.forwarda <= "10";
        elsif id_ex.rs1 = mem_wb.rd and mem_wb.rd_en = '1' then
            control.forwarda <= "01";
        elsif id_ex.rs1 = wb_bp.rd and wb_bp.rd_en = '1' then
            control.forwarda <= "11";
        else
            control.forwarda <= "00";
        end if;
        if id_ex.rs2 = ex_mem.rd and ex_mem.rd_en = '1' then
            control.forwardb <= "10";
        elsif id_ex.rs2 = mem_wb.rd and mem_wb.rd_en = '1' then
            control.forwardb <= "01";
        elsif id_ex.rs2 = wb_bp.rd and wb_bp.rd_en = '1' then
            control.forwardb <= "11";
        else
            control.forwardb <= "00";
        end if;
    end process;


    
    --
    -- IF/ID stage: instruction decode block
    --
   
    process (I_clk, I_areset, I_instr_response, control, id_ex) is
    variable opcode_v : std_logic_vector(6 downto 0);
    variable func3_v : std_logic_vector(2 downto 0);
    variable func7_v : std_logic_vector(6 downto 0);
    variable imm_u_v : data_type;
    variable imm_j_v : data_type;
    variable imm_i_v : data_type;
    variable imm_b_v : data_type;
    variable imm_s_v : data_type;
    variable imm_shamt_v : data_type;
    variable rs1_v, rs2_v, rd_v : reg_type;
    variable selrs1_v : integer range 0 to NUMBER_OF_REGISTERS-1;
    variable selrs2_v : integer range 0 to NUMBER_OF_REGISTERS-1;
    variable uses_rs1_v, uses_rs2_v : boolean;
    begin


        -- Get the opcode and destination register
        opcode_v := I_instr_response.instr(6 downto 0);
        rd_v := I_instr_response.instr(11 downto 7);

        -- Registers to select
        rs1_v := I_instr_response.instr(19 downto 15);
        rs2_v := I_instr_response.instr(24 downto 20);

        -- Get function (extends the opcode)
        func3_v := I_instr_response.instr(14 downto 12);
        func7_v := I_instr_response.instr(31 downto 25);

        -- Create all immediate formats
        imm_u_v(31 downto 12) := I_instr_response.instr(31 downto 12);
        imm_u_v(11 downto 0) := (others => '0');
        
        imm_j_v(31 downto 21) := (others => I_instr_response.instr(31));
        imm_j_v(20 downto 1) := I_instr_response.instr(31) & I_instr_response.instr(19 downto 12) & I_instr_response.instr(20) & I_instr_response.instr(30 downto 21);
        imm_j_v(0) := '0';

        imm_i_v(31 downto 12) := (others => I_instr_response.instr(31));
        imm_i_v(11 downto 0) := I_instr_response.instr(31 downto 20);
        
        imm_b_v(31 downto 13) := (others => I_instr_response.instr(31));
        imm_b_v(12 downto 1) := I_instr_response.instr(31) & I_instr_response.instr(7) & I_instr_response.instr(30 downto 25) & I_instr_response.instr(11 downto 8);
        imm_b_v(0) := '0';

        imm_s_v(31 downto 12) := (others => I_instr_response.instr(31));
        imm_s_v(11 downto 0) := I_instr_response.instr(31 downto 25) & I_instr_response.instr(11 downto 7);
        
        imm_shamt_v(31 downto 5) := (others => '0');
        imm_shamt_v(4 downto 0) := rs2_v;

        -- Select registers from the register file
        if_id.selrs1 <= to_integer(unsigned(rs1_v));
        if_id.selrs2 <= to_integer(unsigned(rs2_v));
        
        -- Load-hazard detection: if current executing instruction is a load and
        -- one of the sources in the ID/EX stage is the destination in the 
        -- EX/MEM stage, only for a load in the EX/MAM stage. 
        -- Note that I-type, U-type and J-type instructions do NOT have an RS2 field,
        -- so this may give a false positve.
        -- Check if instruction uses RS1 and/or RS2 field
        -- Commented out: no real performance benefit
        uses_rs1_v := true; --opcode_v /= "0110111" and opcode_v /= "0010111" and opcode_v /= "1101111" and opcode_v /= "0001111";
        uses_rs2_v := true; --opcode_v = "0110011" or opcode_v = "0100011" or opcode_v = "1100011";
        -- Check for stall
        if id_ex.isload = '1' and id_ex.rd /= "00000" and ((id_ex.rd = rs1_v and uses_rs1_v) or (id_ex.rd = rs2_v and uses_rs2_v)) then
            control.loadhazard <= '1';
        else
            control.loadhazard <= '0';
        end if;
        
        if I_areset = '1' then
            id_ex.pc <= (others => '0');
            id_ex.instr <= (others => 'X');
            id_ex.rd <= (others => '0');
            id_ex.rs1 <= (others => '0');
            id_ex.rs2 <= (others => '0');
            id_ex.rd_en <= '0';
            id_ex.imm <= (others => '0');
            id_ex.isimm <= '0';
            id_ex.isunsigned <= '0';
            id_ex.isload <= '0';
            id_ex.isstore <= '0';
            id_ex.md_op <= (others => '0');
            id_ex.md_start <= '0';
            id_ex.alu_op <= alu_unknown;
            id_ex.pc_op <= pc_incr;
            id_ex.memaccess <= memaccess_nop;
            id_ex.memsize <= memsize_unknown;
            id_ex.csr_op <= csr_nop;
            id_ex.csr_addr <= (others => '0');
            id_ex.csr_immrs1 <= (others => '0');
            control.ecall_request <= '0';
            control.ebreak_request <= '0';
            control.mret_request <= '0';
            control.wfi_request <= '0';
            control.illegal_instruction_decode <= '0';
        elsif rising_edge(I_clk) then
            -- For simulation only
            id_ex.instr <= I_instr_response.instr;
            if I_sreset = '1' then
                id_ex.pc <= (others => '0');
                id_ex.instr <= (others => 'X');
                id_ex.rd <= (others => '0');
                id_ex.rs1 <= (others => '0');
                id_ex.rs2 <= (others => '0');
                id_ex.rd_en <= '0';
                id_ex.imm <= (others => '0');
                id_ex.isimm <= '0';
                id_ex.isunsigned <= '0';
                id_ex.isload <= '0';
                id_ex.isstore <= '0';
                id_ex.md_op <= (others => '0');
                id_ex.md_start <= '0';
                id_ex.alu_op <= alu_unknown;
                id_ex.pc_op <= pc_incr;
                id_ex.memaccess <= memaccess_nop;
                id_ex.memsize <= memsize_unknown;
                id_ex.csr_op <= csr_nop;
                id_ex.csr_addr <= (others => '0');
                id_ex.csr_immrs1 <= (others => '0');
                control.ecall_request <= '0';
                control.ebreak_request <= '0';
                control.mret_request <= '0';
                control.wfi_request <= '0';
                control.illegal_instruction_decode <= '0';
            else
                -- If in debug...
                if control.stall_on_debug = '1' then
                    -- Set all registers to default
                    id_ex.rd <= (others => '0');
                    id_ex.rs1 <= (others => '0');
                    id_ex.rs2 <= (others => '0');
                    id_ex.rd_en <= '0';
                    id_ex.imm <= imm_i_v;
                    id_ex.isimm <= '0';
                    id_ex.isunsigned <= '0';
                    id_ex.isload <= '0';
                    id_ex.isstore <= '0';
                    id_ex.alu_op <= alu_nop;
                    id_ex.pc_op <= pc_incr;
                    id_ex.md_start <= '0';
                    id_ex.md_op <= (others => '0');
                    id_ex.memaccess <= memaccess_nop;
                    id_ex.memsize <= memsize_unknown;
                    id_ex.csr_op <= csr_nop;
                    id_ex.csr_addr <= (others => '0');
                    id_ex.csr_immrs1 <= (others => '0');
                    control.ecall_request <= '0';
                    control.ebreak_request <= '0';
                    control.mret_request <= '0';
                    control.wfi_request <= '0';
                    control.illegal_instruction_decode <= '0';
                -- If a trap is requested
                elsif control.trap_request = '1' then
                    -- ALU does nothing
                    id_ex.alu_op <= alu_nop;
                    -- No writeback to register
                    id_ex.rd <= (others => '0');
                    id_ex.rd_en <= '0';
                    -- Load PC with MTVEC CSR
                    id_ex.pc_op <= pc_load_mtvec;
                    -- Disable CSR operation
                    id_ex.csr_op <= csr_nop;
                    -- Do not start the MD unit
                    id_ex.md_start <= '0';
                    -- ECALL request reset
                    control.ecall_request <= '0';
                    -- EBREAK request reset
                    control.ebreak_request <= '0';
                    -- WFI request reset
                    control.wfi_request <= '0';
                    -- Illegal instruction reset
                    control.illegal_instruction_decode <= '0';
                    -- Disable memory operation
                    id_ex.isload <= '0';
                    id_ex.isstore <= '0';

                -- Check if we have a stall, but not due to load hazard
                -- Stall here only for MD operation. A load hazard will
                -- trigger a flush.
                elsif control.stall = '1' and control.loadhazard = '0' then
                    -- Set id_ex.md_start to 0. It is already registered.
                    id_ex.md_start <= '0';
                    -- If the MD unit is ready and we are still doing MD operation,
                    -- load the data in the selected register. MD operation cannot
                    -- be interrupted by trap.
                    if md.ready = '1' then
                        id_ex.pc_op <= pc_incr;
                        id_ex.rd_en <= '1';
                    end if;
                    control.wfi_request <= '0';
                else
                    -- Set all registers to default
                    id_ex.pc <= if_id.pc;
                    id_ex.rd <= rd_v;
                    id_ex.rs1 <= rs1_v;
                    id_ex.rs2 <= rs2_v;
                    id_ex.rd_en <= '0';
                    id_ex.imm <= imm_i_v;
                    id_ex.isimm <= '0';
                    id_ex.isunsigned <= '0';
                    id_ex.isload <= '0';
                    id_ex.isstore <= '0';
                    id_ex.md_op <= (others => '0');
                    id_ex.md_start <= '0';
                    id_ex.alu_op <= alu_nop;
                    id_ex.pc_op <= pc_incr;
                    id_ex.memaccess <= memaccess_nop;
                    id_ex.memsize <= memsize_unknown;
                    id_ex.csr_op <= csr_nop;
                    id_ex.csr_addr <= imm_i_v(11 downto 0); -- always assign
                    id_ex.csr_immrs1 <= rs1_v; -- always assign
                    control.ecall_request <= '0';
                    control.ebreak_request <= '0';
                    control.mret_request <= '0';
                    control.wfi_request <= '0';
                    control.illegal_instruction_decode <= '0';

                    -- If we flush the pipeline, don't execute the instruction
                    -- Also for use-after-load (load hazard)
                    if control.flush = '1' then
                        -- Keep default values
                        null;
                    -- Check for only 32-bit instructions
                    elsif opcode_v(1 downto 0) /= "11" then
                        control.illegal_instruction_decode <= '1';
                    else
                        -- Check remaining opcode bits
                        case opcode_v(6 downto 2) is
                            -- LUI
                            when "01101" =>
                                id_ex.alu_op <= alu_lui;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_u_v;
                                id_ex.isimm <= '1';
                            -- AUIPC
                            when "00101" =>
                                id_ex.alu_op <= alu_auipc;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_u_v;
                                id_ex.isimm <= '1';
                            -- JAL
                            when "11011" =>
                                id_ex.alu_op <= alu_jal_jalr;
                                id_ex.pc_op <= pc_loadoffset;
                                id_ex.rd_en <= '1';
                                id_ex.imm <= imm_j_v;
                            -- JALR
                            when "11001" =>
                                if func3_v = "000" then
                                    id_ex.alu_op <= alu_jal_jalr;
                                    id_ex.pc_op <= pc_loadoffsetregister;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_i_v;
                                else
                                    control.illegal_instruction_decode <= '1';
                                end if;
                            -- Branches
                            when "11000" =>
                                -- Set the registers to compare. Comparison is handled by the ALU.
                                id_ex.imm <= imm_b_v;
                                id_ex.pc_op <= pc_branch;
                                case func3_v is
                                    when "000" => id_ex.alu_op <= alu_beq;
                                    when "001" => id_ex.alu_op <= alu_bne;
                                    when "100" => id_ex.alu_op <= alu_blt;
                                    when "101" => id_ex.alu_op <= alu_bge;
                                    when "110" => id_ex.alu_op <= alu_bltu; id_ex.isunsigned <= '1';
                                    when "111" => id_ex.alu_op <= alu_bgeu; id_ex.isunsigned <= '1';
                                    when others =>
                                        -- Reset defaults
                                        id_ex.pc_op <= pc_incr;
                                        control.illegal_instruction_decode <= '1';
                                end case;

                            -- Arithmetic/logic register/immediate
                            when "00100" =>
                                -- ADDI
                                if func3_v = "000" then
                                    id_ex.alu_op <= alu_addi;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_i_v;
                                    id_ex.isimm <= '1';
                                -- SLTI
                                elsif func3_v = "010" then
                                    id_ex.alu_op <= alu_slti;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_i_v;
                                    id_ex.isimm <= '1';
                                -- SLTIU
                                elsif func3_v = "011" then
                                    id_ex.alu_op <= alu_sltiu;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_i_v;
                                    id_ex.isimm <= '1';
                                    id_ex.isunsigned <= '1';
                                -- XORI
                                elsif func3_v = "100" then
                                    id_ex.alu_op <= alu_xori;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_i_v;
                                    id_ex.isimm <= '1';
                                -- ORI
                                elsif func3_v = "110" then
                                    id_ex.alu_op <= alu_ori;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_i_v;
                                    id_ex.isimm <= '1';
                                -- ANDI
                                elsif func3_v = "111" then
                                    id_ex.alu_op <= alu_andi;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_i_v;
                                    id_ex.isimm <= '1';
                                -- SLLI
                                elsif func3_v = "001" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_slli;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_shamt_v;
                                    id_ex.isimm <= '1';
                                -- SRLI
                                elsif func3_v = "101" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_srli;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_shamt_v;
                                    id_ex.isimm <= '1';
                                -- SRAI
                                elsif func3_v = "101" and func7_v = "0100000" then
                                    id_ex.alu_op <= alu_srai;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_shamt_v;
                                    id_ex.isimm <= '1';
                                -- BCLRI
                                elsif func3_v = "001" and func7_v = "0100100" and HAVE_ZBS then
                                    id_ex.alu_op <= alu_bclri;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_shamt_v;
                                    id_ex.isimm <= '1';
                                -- BEXTI
                                elsif func3_v = "101" and func7_v = "0100100" and HAVE_ZBS then
                                    id_ex.alu_op <= alu_bexti;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_shamt_v;
                                    id_ex.isimm <= '1';
                                -- BINVI
                                elsif func3_v = "001" and func7_v = "0110100" and HAVE_ZBS then
                                    id_ex.alu_op <= alu_binvi;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_shamt_v;
                                    id_ex.isimm <= '1';
                                 -- BSETI
                                 elsif func3_v = "001" and func7_v = "0010100" and HAVE_ZBS then
                                    id_ex.alu_op <= alu_bseti;
                                    id_ex.rd_en <= '1';
                                    id_ex.imm <= imm_shamt_v;
                                    id_ex.isimm <= '1';
                                else
                                    control.illegal_instruction_decode <= '1';
                                end if;

                            -- Arithmetic/logic register/register
                            when "01100" =>
                                -- ADD
                                if func3_v = "000" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_add;
                                    id_ex.rd_en <= '1';
                                -- SUB
                                elsif func3_v = "000" and func7_v = "0100000" then
                                    id_ex.alu_op <= alu_sub;
                                    id_ex.rd_en <= '1';
                                -- SLL
                                elsif func3_v = "001" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_sll; 
                                    id_ex.rd_en <= '1';
                                -- SLT
                                elsif func3_v = "010" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_slt; 
                                    id_ex.rd_en <= '1';
                                -- SLTU
                                elsif func3_v = "011" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_sltu; 
                                    id_ex.rd_en <= '1';
                                    id_ex.isunsigned <= '1';
                                -- XOR
                                elsif func3_v = "100" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_xor; 
                                    id_ex.rd_en <= '1';
                                -- SRL
                                elsif func3_v = "101" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_srl; 
                                    id_ex.rd_en <= '1';
                                -- SRA
                                elsif func3_v = "101" and func7_v = "0100000" then
                                    id_ex.alu_op <= alu_sra; 
                                    id_ex.rd_en <= '1';
                                -- OR
                                elsif func3_v = "110" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_or;
                                    id_ex.rd_en <= '1';
                                -- AND
                                elsif func3_v = "111" and func7_v = "0000000" then
                                    id_ex.alu_op <= alu_and;
                                    id_ex.rd_en <= '1';
                                 -- SH1ADD
                                elsif func3_v = "010" and func7_v = "0010000" and HAVE_ZBA then
                                    id_ex.alu_op <= alu_sh1add;
                                    id_ex.rd_en <= '1';
                                -- SH2ADD
                                elsif func3_v = "100" and func7_v = "0010000" and HAVE_ZBA then
                                    id_ex.alu_op <= alu_sh2add;
                                    id_ex.rd_en <= '1';
                                -- SH3ADD
                                elsif func3_v = "110" and func7_v = "0010000" and HAVE_ZBA then
                                    id_ex.alu_op <= alu_sh3add;
                                    id_ex.rd_en <= '1';
                                -- BCLR
                                elsif func3_v = "001" and func7_v = "0100100" and HAVE_ZBS then
                                    id_ex.alu_op <= alu_bclr;
                                    id_ex.rd_en <= '1';
                                -- BEXT
                                elsif func3_v = "101" and func7_v = "0100100" and HAVE_ZBS then
                                    id_ex.alu_op <= alu_bext;
                                    id_ex.rd_en <= '1';
                                -- BINV
                                elsif func3_v = "001" and func7_v = "0110100" and HAVE_ZBS then
                                    id_ex.alu_op <= alu_binv;
                                    id_ex.rd_en <= '1';
                                -- BSET
                                elsif func3_v = "001" and func7_v = "0010100" and HAVE_ZBS then
                                    id_ex.alu_op <= alu_bset;
                                    id_ex.rd_en <= '1';
                                -- CZERO.EQZ
                                elsif func3_v = "101" and func7_v = "0000111" and HAVE_ZICOND then
                                    id_ex.alu_op <= alu_czeroeqz;
                                    id_ex.rd_en <= '1';
                                -- CZERO.NEZ
                                elsif func3_v = "111" and func7_v = "0000111" and HAVE_ZICOND then
                                    id_ex.alu_op <= alu_czeronez;
                                    id_ex.rd_en <= '1';
                               -- Multiply, divide, remainder
                                elsif func7_v = "0000001" then
                                    -- Set operation to multiply or divide/remainder
                                    -- func3 contains the real operation
                                    if HAVE_MULDIV then
                                        case func3_v(2) is
                                            when '0' => id_ex.alu_op <= alu_multiply;
                                            when '1' => id_ex.alu_op <= alu_divrem;
                                            when others => null;
                                        end case;
                                        -- Hold the PC
                                        id_ex.pc_op <= pc_hold;
                                        -- func3 contains the function
                                        id_ex.md_op <= func3_v;
                                        -- Start multiply/divide/remainder
                                        id_ex.md_start <= '1';
                                    else
                                        control.illegal_instruction_decode <= '1';
                                    end if;
                                else
                                    control.illegal_instruction_decode <= '1';
                                end if;

                            -- S(W|H|B)
                            when "01000" =>
                                case func3_v is
                                    -- Store byte (no sign extension or zero extension)
                                    when "000" =>
                                        id_ex.alu_op <= alu_sb;
                                        id_ex.memaccess <= memaccess_write;
                                        id_ex.memsize <= memsize_byte;
                                        id_ex.imm <= imm_s_v;
                                        id_ex.isimm <= '1';
                                        id_ex.isstore <= '1';
                                    -- Store halfword (no sign extension or zero extension)
                                    when "001" =>
                                        id_ex.alu_op <= alu_sh;
                                        id_ex.memaccess <= memaccess_write;
                                        id_ex.memsize <= memsize_halfword;
                                        id_ex.imm <= imm_s_v;
                                        id_ex.isimm <= '1';
                                        id_ex.isstore <= '1';
                                        -- Store word (no sign extension or zero extension)
                                    when "010" =>
                                        id_ex.alu_op <= alu_sw;
                                        id_ex.memaccess <= memaccess_write;
                                        id_ex.memsize <= memsize_word;
                                        id_ex.imm <= imm_s_v;
                                        id_ex.isimm <= '1';
                                        id_ex.isstore <= '1';
                                    when others =>
                                        control.illegal_instruction_decode <= '1';
                                end case;
                                
                            -- L{W|H|B|HU|BU}
                            when "00000" =>
                                case func3_v is
                                    -- LB
                                    when "000" =>
                                        id_ex.alu_op <= alu_lb;
                                        id_ex.rd_en <= '1';
                                        id_ex.memaccess <= memaccess_read;
                                        id_ex.memsize <= memsize_byte;
                                        id_ex.imm <= imm_i_v;
                                        id_ex.isimm <= '1';
                                        id_ex.isload <= '1';
                                    -- LH
                                    when "001" =>
                                        id_ex.alu_op <= alu_lh;
                                        id_ex.rd_en <= '1';
                                        id_ex.memaccess <= memaccess_read;
                                        id_ex.memsize <= memsize_halfword;
                                        id_ex.imm <= imm_i_v;
                                        id_ex.isimm <= '1';
                                        id_ex.isload <= '1';
                                    -- LW
                                    when "010" =>
                                        id_ex.alu_op <= alu_lw;
                                        id_ex.rd_en <= '1';
                                        id_ex.memaccess <= memaccess_read;
                                        id_ex.memsize <= memsize_word;
                                        id_ex.imm <= imm_i_v;
                                        id_ex.isimm <= '1';
                                        id_ex.isload <= '1';
                                    -- LBU
                                    when "100" =>
                                        id_ex.alu_op <= alu_lbu;
                                        id_ex.rd_en <= '1';
                                        id_ex.memaccess <= memaccess_read;
                                        id_ex.memsize <= memsize_byte;
                                        id_ex.imm <= imm_i_v;
                                        id_ex.isimm <= '1';
                                        id_ex.isload <= '1';
                                    -- LHU
                                    when "101" =>
                                        id_ex.alu_op <= alu_lhu;
                                        id_ex.rd_en <= '1';
                                        id_ex.memaccess <= memaccess_read;
                                        id_ex.memsize <= memsize_halfword;
                                        id_ex.imm <= imm_i_v;
                                        id_ex.isimm <= '1';
                                        id_ex.isload <= '1';
                                    when others =>
                                        control.illegal_instruction_decode <= '1';
                                end case;

                            -- CSR, ECALL, EBREAK, MRET, WFI
                            when "11100" =>
                                case func3_v is
                                    when "000" =>
                                        -- ECALL/EBREAK/MRET/WFI
                                        if I_instr_response.instr(31 downto 20) = "000000000000" then
                                            -- ECALL
                                            control.ecall_request <= '1';
                                            id_ex.alu_op <= alu_trap;
                                            id_ex.pc_op <= pc_hold;
                                        elsif I_instr_response.instr(31 downto 20) = "000000000001" then
                                            -- EBREAK
                                            control.ebreak_request <= '1';
                                            -- Check the dcsr.ebreakm flag, if 0 then trap
                                            if csr_reg.dcsr(15) = '0' then
                                                id_ex.alu_op <= alu_trap;
                                                id_ex.pc_op <= pc_hold;
                                            end if;
                                        elsif I_instr_response.instr(31 downto 20) = "001100000010" then
                                            -- MRET
                                            id_ex.alu_op <= alu_mret;
                                            control.mret_request <= '1';
                                            id_ex.pc_op <= pc_load_mepc;
                                        elsif I_instr_response.instr(31 downto 20) = "000100000101" then
                                            -- WFI
                                            -- Only execute while not stepping
                                            control.wfi_request <= not control.isstepping;
                                        else
                                            control.illegal_instruction_decode <= '1';
                                        end if;
                                
                                    when "001" =>
                                        id_ex.alu_op <= alu_csr;
                                        id_ex.csr_op <= csr_rw;
                                        id_ex.rd_en <= '1';
                                        id_ex.csr_addr <= imm_i_v(11 downto 0);
                                        id_ex.csr_immrs1 <= rs1_v; -- RS1
                                    when "010" =>
                                        id_ex.alu_op <= alu_csr;
                                        id_ex.csr_op <= csr_rs;
                                        id_ex.rd_en <= '1';
                                        id_ex.csr_addr <= imm_i_v(11 downto 0);
                                        id_ex.csr_immrs1 <= rs1_v; -- RS1
                                    when "011" =>
                                        id_ex.alu_op <= alu_csr;
                                        id_ex.csr_op <= csr_rc;
                                        id_ex.rd_en <= '1';
                                        id_ex.csr_addr <= imm_i_v(11 downto 0);
                                        id_ex.csr_immrs1 <= rs1_v; -- RS1
                                    when "101" =>
                                        id_ex.alu_op <= alu_csr;
                                        id_ex.csr_op <= csr_rwi;
                                        id_ex.rd_en <= '1';
                                        id_ex.csr_addr <= imm_i_v(11 downto 0);
                                        id_ex.csr_immrs1 <= rs1_v; -- imm
                                    when "110" =>
                                        id_ex.alu_op <= alu_csr;
                                        id_ex.csr_op <= csr_rsi;
                                        id_ex.rd_en <= '1';
                                        id_ex.csr_addr <= imm_i_v(11 downto 0);
                                        id_ex.csr_immrs1 <= rs1_v; -- imm
                                    when "111" =>
                                        id_ex.alu_op <= alu_csr;
                                        id_ex.csr_op <= csr_rci;
                                        id_ex.rd_en <= '1';
                                        id_ex.csr_addr <= imm_i_v(11 downto 0);
                                        id_ex.csr_immrs1 <= rs1_v; -- imm
                                    when others =>
                                        control.illegal_instruction_decode <= '1';
                                end case;


                            -- FENCE, FENCE.I
                            when "00011" =>
                                if func3_v = "000" or func3_v = "001" then
                                    -- Just ignore
                                    null;
                                else
                                    control.illegal_instruction_decode <= '1';
                                end if;
                                
                            -- Illegal instruction or not implemented
                            when others =>
                                control.illegal_instruction_decode <= '1';
                        end case;
                        
                        -- If the destination register is x0, block any write
                        -- to it. x0 (zero) is loaded with all null-bits at
                        -- reconfiguration time and is never written. So the
                        -- contents is always null-bits.
                        if rd_v = "00000" then
                            id_ex.rd_en <= '0';
                        end if;
                   end if; -- flush
                end if; -- stall
            end if; -- sreset
        end if; -- rising_edge
            
    end process;

    --
    -- Registers
    --
    -- Register RS1
    process (I_clk, mem_wb, control, I_dm_core_data_request) is
    variable rd_v : integer range 0 to NUMBER_OF_REGISTERS;
    begin
        -- Select destination register
        if control.indebug = '1' then
            rd_v := to_integer(unsigned(I_dm_core_data_request.address(4 downto 0)));
        else
            rd_v := to_integer(unsigned(mem_wb.rd));
        end if;
        -- No asynchronous reset!
        if rising_edge(I_clk) then
            -- Write register if Rd has valid data
            if control.indebug = '0' and mem_wb.rd_en = '1' then
                regs_rs1(rd_v) <= mem_wb.rddata;
            -- Write via debugger
            elsif control.stall_on_debug = '1' and I_dm_core_data_request.writegpr = '1' and rd_v /= 0 then
                regs_rs1(rd_v) <= I_dm_core_data_request.data;
            end if;
            -- Fetch register contents
            id_ex.rs1data <= regs_rs1(if_id.selrs1);
        end if;
    end process;
    -- Register RS2
    process (I_clk, mem_wb, control, I_dm_core_data_request) is
    variable rd_v : integer range 0 to NUMBER_OF_REGISTERS;
    begin
        -- Select destination register
        if control.indebug = '1' then
            rd_v := to_integer(unsigned(I_dm_core_data_request.address(4 downto 0)));
        else
            rd_v := to_integer(unsigned(mem_wb.rd));
        end if;
        -- No asynchronous reset!
        if rising_edge(I_clk) then
            -- Write register if Rd has valid data
            if control.indebug = '0' and mem_wb.rd_en = '1' then
                regs_rs2(rd_v) <= mem_wb.rddata;
            -- Write via debugger
            elsif control.stall_on_debug = '1' and I_dm_core_data_request.writegpr = '1' and rd_v /= 0 then
                regs_rs2(rd_v) <= I_dm_core_data_request.data;
            end if;
            -- Fetch register contents
            id_ex.rs2data <= regs_rs2(if_id.selrs2);
        end if;
    end process;
    
    -- Register debug
    -- Needed because RS1 and RS2 get their read register number from selrs1 and selrs2
    debuginramgen: if HAVE_OCD generate
        process (I_clk, mem_wb, control, I_dm_core_data_request) is
        variable rd_v : integer range 0 to NUMBER_OF_REGISTERS;
        begin
            -- Select destination register
            if control.indebug = '1' then
                rd_v := to_integer(unsigned(I_dm_core_data_request.address(4 downto 0)));
            else
                rd_v := to_integer(unsigned(mem_wb.rd));
            end if;
            -- No asynchronous reset!
            if rising_edge(I_clk) then
                -- Write register if Rd has valid data
                if control.indebug = '0' and mem_wb.rd_en = '1' then
                    regs_dbg(rd_v) <= mem_wb.rddata;
                -- Write via debugger
                elsif control.stall_on_debug = '1' and I_dm_core_data_request.writegpr = '1' and rd_v /= 0 then
                    regs_dbg(rd_v) <= I_dm_core_data_request.data;
                end if;
                -- Fetch register contents
                data_from_gpr <= regs_dbg(rd_v);
            end if;
        end process;
    end generate;
    debuginramnotgen: if not HAVE_OCD generate
        data_from_gpr <= all_zeros_c;
    end generate;


  
    
    
    --
    -- Instruction execute block
    --
    
    process (I_clk, I_areset, id_ex, ex_mem, mem_wb, wb_bp, control, md, csr_access) is
    variable al_v, bl_v : std_logic_vector(32 downto 0);
    variable a_v, b_v, r_v : data_type;
    variable rs2_pre_imm_v : data_type;
    variable cmpeq_v, cmplt_v : std_logic;
    variable signs_v : data_type;
    variable bitsft_v : data_type;
    variable valid_v : std_logic;
    begin
        -- Get data
        if control.forwarda = "10" then
            a_v := ex_mem.rs1data;
        elsif control.forwarda = "01" then
            a_v := mem_wb.rddata;
        elsif control.forwarda = "11" then
            a_v := wb_bp.rddata;
        else
            a_v := id_ex.rs1data;
        end if;

        if control.forwardb = "10" then
            b_v := ex_mem.rs1data; -- !note: from register RS1, this contains the ALU result
        elsif control.forwardb = "01" then
            b_v := mem_wb.rddata;
        elsif control.forwardb = "11" then
            b_v := wb_bp.rddata;
        else
            b_v := id_ex.rs2data;
        end if;
        
        -- Make a copy for RS2, before immediate remapping
        rs2_pre_imm_v := b_v;
        
        -- Immediate instruction?
        if id_ex.isimm = '1' then
            b_v := id_ex.imm;
        end if;
        
        -- For debugging, can be removed
        --synthesis translate_off
        id_ex.a <= a_v;
        id_ex.b <= b_v;
        id_ex.c <= rs2_pre_imm_v;
        --synthesis translate_on
        
        -- Expand with sign bit or zero expand, for less-than comparison
        al_v := (a_v(a_v'left) and (not id_ex.isunsigned)) & a_v;
        bl_v := (b_v(b_v'left) and (not id_ex.isunsigned)) & b_v;

        -- Comparison
        if a_v = b_v then
            cmpeq_v := '1';
        else
            cmpeq_v := '0';
        end if;
        if signed(al_v) < signed(bl_v) then
            cmplt_v := '1';
        else
            cmplt_v := '0';
        end if;
        
        -- Reset penaly
        control.penalty <= '0';
        -- Reset valid
        valid_v := '0';

        -- Clear result
        r_v := all_zeros_c;
        
        case id_ex.alu_op is
        
            -- Default no-op
            when alu_unknown | alu_nop | alu_trap =>
                null;
            
           -- Return from trap, dirty trick, needs to be revised.
           -- This sets the branch penalty, which causes a flush
            when alu_mret =>
                control.penalty <= '1';
                valid_v := '1';

            -- Add, sub and logic, load and store
            when alu_add | alu_addi| alu_sb | alu_sh | alu_sw | alu_lb | alu_lbu | alu_lh | alu_lhu | alu_lw =>
                r_v := std_logic_vector(unsigned(a_v) + unsigned(b_v));
                valid_v := '1';
            when alu_sh1add | alu_sh2add | alu_sh3add =>
                if HAVE_ZBA then
                    if id_ex.alu_op = alu_sh1add then
                        a_v := a_v(a_v'left-1 downto 0) & '0';
                    elsif id_ex.alu_op = alu_sh2add then
                        a_v := a_v(a_v'left-2 downto 0) & "00";
                    elsif id_ex.alu_op = alu_sh3add then
                        a_v := a_v(a_v'left-3 downto 0) & "000";
                    end if;
                    r_v := std_logic_vector(unsigned(a_v) + unsigned(b_v));
                end if;
            when alu_sub =>
                r_v := std_logic_vector(unsigned(a_v) - unsigned(b_v));
                valid_v := '1';
            when alu_and | alu_andi =>
                r_v := a_v and b_v;
                valid_v := '1';
            when alu_or | alu_ori =>
                r_v := a_v or b_v;
                valid_v := '1';
            when alu_xor | alu_xori =>
                r_v := a_v xor b_v;
                valid_v := '1';
            when alu_czeroeqz =>
                if HAVE_ZICOND then
                    if b_v = all_zeros_c then
                        r_v := all_zeros_c;
                    else
                        r_v := a_v;
                    end if;
                end if;
            when alu_czeronez =>
                if HAVE_ZICOND then
                    if b_v /= all_zeros_c then
                        r_v := all_zeros_c;
                    else
                        r_v := a_v;
                    end if;
                end if;

            -- Bit instructions
            when alu_bclr | alu_bclri =>
                if HAVE_ZBS then
                    bitsft_v := (others => '0');
                    bitsft_v(to_integer(unsigned(b_v(4 downto 0)))) := '1';
                    r_v := a_v and not bitsft_v;
                 end if;
            when alu_binv | alu_binvi =>
                if HAVE_ZBS then
                    bitsft_v := (others => '0');
                    bitsft_v(to_integer(unsigned(b_v(4 downto 0)))) := '1';
                    r_v := a_v xor bitsft_v;
                 end if;
            when alu_bset | alu_bseti =>
                if HAVE_ZBS then
                    bitsft_v := (others => '0');
                    bitsft_v(to_integer(unsigned(b_v(4 downto 0)))) := '1';
                    r_v := a_v or bitsft_v;
                 end if;
            when alu_bext | alu_bexti =>
                if HAVE_ZBS then
                    if a_v(to_integer(unsigned(b_v(4 downto 0)))) = '1' then
                        r_v(0) := '1';
                    end if;
                 end if;

            -- Set if less-than
            when alu_slt | alu_sltu | alu_slti | alu_sltiu =>
                r_v(0) := cmplt_v;
                valid_v := '1';

            -- Shifts et al
            -- Shift left
            when alu_sll | alu_slli =>
                signs_v := all_zeros_c;

                if b_v(4) = '1' then
                    a_v := a_v(a_v'left-16 downto 0) & signs_v(31 downto 16);
                    signs_v := signs_v(15 downto 0) & all_zeros_c(15 downto 0);
                end if;
                if b_v(3) = '1' then
                    a_v := a_v(a_v'left-8 downto 0) & signs_v(31 downto 24);
                    signs_v := signs_v(7 downto 0) & all_zeros_c(23 downto 0);
                end if;
                if b_v(2) = '1' then
                    a_v := a_v(a_v'left-4 downto 0) & signs_v(31 downto 28);
                    signs_v := signs_v(3 downto 0) & all_zeros_c(27 downto 0);
                end if;
                if b_v(1) = '1' then
                    a_v := a_v(a_v'left-2 downto 0) & signs_v(31 downto 30);
                    signs_v := signs_v(1 downto 0) & all_zeros_c(29 downto 0);
                end if;
                if b_v(0) = '1' then
                    a_v := a_v(a_v'left-1 downto 0) & signs_v(31 downto 31);
                end if;
                r_v := a_v;
                valid_v := '1';
            -- Shift right
            when alu_sra | alu_srl | alu_srai | alu_srli =>
                if id_ex.alu_op = alu_srl or id_ex.alu_op = alu_srli then
                    signs_v := all_zeros_c;
                else
                    signs_v := (others => a_v(a_v'left));
                end if;
                if b_v(4) = '1' then
                    a_v := signs_v(15 downto 0) & a_v(a_v'left downto 16);
                    signs_v := all_zeros_c(15 downto 0) & signs_v(31 downto 16);
                end if;
                if b_v(3) = '1' then
                    a_v := signs_v(7 downto 0) & a_v(a_v'left downto 8);
                    signs_v := all_zeros_c(7 downto 0) & signs_v(31 downto 8);
                end if;
                if b_v(2) = '1' then
                    a_v := signs_v(3 downto 0) & a_v(a_v'left downto 4);
                    signs_v := all_zeros_c(3 downto 0) & signs_v(31 downto 4);
                end if;
                if b_v(1) = '1' then
                    a_v := signs_v(1 downto 0) & a_v(a_v'left downto 2);
                    signs_v := all_zeros_c(1 downto 0) & signs_v(31 downto 2);
                end if;
                if b_v(0) = '1' then
                    a_v := signs_v(0 downto 0) & a_v(a_v'left downto 1);
                end if;
                r_v := a_v;
                valid_v := '1';

            when alu_lui =>
                r_v := b_v;
                valid_v := '1';
            when alu_auipc =>
                r_v := std_logic_vector(unsigned(id_ex.pc) + unsigned(b_v)) ;
                valid_v := '1';
            -- Memory loads are not handled by the ALU. see EX/MEM stage

            -- Jumps and calls
            when alu_jal_jalr =>
                r_v := std_logic_vector(unsigned(id_ex.pc) + 4);
                control.penalty <= '1';
                valid_v := '1';
                
            -- Branches
            when alu_beq =>
                r_v := (others => '0');
                r_v(0) := cmpeq_v;
                control.penalty <= cmpeq_v;
                valid_v := '1';
            when alu_bne =>
                r_v := (others => '0');
                r_v(0) := not cmpeq_v;
                control.penalty <= not cmpeq_v;
                valid_v := '1';
            when alu_blt | alu_bltu =>
                r_v := (others => '0');
                r_v(0) := cmplt_v;
                control.penalty <= cmplt_v;
                valid_v := '1';
            when alu_bge | alu_bgeu =>
                r_v := (others => '0');
                r_v(0) := not cmplt_v;
                control.penalty <= not cmplt_v;
                valid_v := '1';

            -- Pass data from multiplier
            when alu_multiply =>
                r_v := md.mul;
                -- Wait for MUL/DIV to complete
                if control.state = state_md2 then valid_v := '1'; else valid_v := '0'; end if;
                
            -- Pass data from divider
            when alu_divrem =>
                r_v := md.div;
                -- Wait for MUL/DIV to complete
                if control.state = state_md2 then valid_v := '1'; else valid_v := '0'; end if;

            -- Pass data from CSR
            when alu_csr =>
                r_v := csr_access.data_from_csr;
                valid_v := '1';

            when others =>
                null;
        end case;
        
        -- For debugging, can be removed
        --synthesis translate_off
        id_ex.r <= r_v;
        id_ex.valid <= valid_v;
        --synthesis translate_on
        
        -- Setup the EX/MEM stage
        if I_areset = '1' then
            ex_mem.rs1 <= (others => '0');
            ex_mem.rs2 <= (others => '0');
            ex_mem.rs1data <= all_zeros_c;
            ex_mem.rs2data <= all_zeros_c;
            ex_mem.rd <= (others => '0');
            ex_mem.rd_en <= '0';
            ex_mem.memaccess <= memaccess_nop;
            ex_mem.memsize <= memsize_unknown;
            ex_mem.isload <= '0';
            ex_mem.isstore <= '0';
            ex_mem.alu_op <= alu_unknown;
            ex_mem.pc <= all_zeros_c;
            ex_mem.valid <= '0';
        elsif rising_edge(I_clk) then
            if I_sreset = '1' then
                ex_mem.rs1 <= (others => '0');
                ex_mem.rs2 <= (others => '0');
                ex_mem.rs1data <= all_zeros_c;
                ex_mem.rs2data <= all_zeros_c;
                ex_mem.rd <= (others => '0');
                ex_mem.rd_en <= '0';
                ex_mem.memaccess <= memaccess_nop;
                ex_mem.memsize <= memsize_unknown;
                ex_mem.isload <= '0';
                ex_mem.isstore <= '0';
                ex_mem.alu_op <= alu_unknown;
                ex_mem.pc <= all_zeros_c;
                ex_mem.valid <= '0';
            else
                -- Follows ID/EX PC, needed for setting correct
                -- return address on load/store trap
                ex_mem.pc <= id_ex.pc;
                -- If there is a trap, cancel next instruction
                -- If we're in debug, cancel instructions
                if control.trap_request = '1' or control.stall_on_debug = '1' then
                    ex_mem.rs1 <= (others => '0');
                    ex_mem.rs2 <= (others => '0');
                    ex_mem.rs1data <= all_zeros_c;
                    ex_mem.rs2data <= all_zeros_c;
                    ex_mem.rd <= (others => '0');
                    ex_mem.rd_en <= '0';
                    ex_mem.memaccess <= memaccess_nop;
                    ex_mem.memsize <= memsize_unknown;
                    ex_mem.isload <= '0';
                    ex_mem.isstore <= '0';
                    ex_mem.alu_op <= alu_nop;
                    ex_mem.valid <= '0';
                else
                    ex_mem.rs1 <= id_ex.rs1;
                    ex_mem.rs2 <= id_ex.rs2;
                    ex_mem.rd <= id_ex.rd;
                    ex_mem.rd_en <= id_ex.rd_en;
                    ex_mem.rs1data <= r_v;
                    ex_mem.rs2data <= rs2_pre_imm_v; -- RS2 after forwarding, before immediate remapping
                    ex_mem.memaccess <= id_ex.memaccess;
                    ex_mem.memsize <= id_ex.memsize;
                    ex_mem.isload <= id_ex.isload;
                    ex_mem.isstore <= id_ex.isstore;
                    ex_mem.alu_op <= id_ex.alu_op;
                    ex_mem.valid <= valid_v;
                end if;
            end if;
        end if;
    end process;

    --
    -- EX/MEM stage: load/store data or pass through
    --
    -- Address in RS1, data (store) in RS2
    process (ex_mem, I_dm_core_data_request, control) is
    begin
        O_bus_request.stb <= ex_mem.isload or ex_mem.isstore or (I_dm_core_data_request.stb and boolean_to_std_logic(HAVE_OCD));

        -- If we are in debug mode ...
        if control.indebug = '1' then
            -- Set read or write (or nop)
            if I_dm_core_data_request.readmem = '1' then
                O_bus_request.acc <= memaccess_read;
            elsif I_dm_core_data_request.writemem = '1' then
                O_bus_request.acc <= memaccess_write;
            else
                O_bus_request.acc <= memaccess_nop;
            end if;
            -- Translate memory size
            if (I_dm_core_data_request.readmem or I_dm_core_data_request.writemem) = '0' then
                O_bus_request.size <= memsize_unknown;
            elsif I_dm_core_data_request.size = "00" then
                O_bus_request.size <= memsize_byte;
            elsif I_dm_core_data_request.size = "01" then
                O_bus_request.size <= memsize_halfword;
            elsif I_dm_core_data_request.size = "10" then
                O_bus_request.size <= memsize_word;
            else
                O_bus_request.size <= memsize_unknown;
            end if;
            O_bus_request.data <= I_dm_core_data_request.data;
            O_bus_request.addr <= I_dm_core_data_request.address;
            csr_transfer.address_to_mtval <= I_dm_core_data_request.address;
        else
            O_bus_request.acc <= ex_mem.memaccess;
            O_bus_request.size <= ex_mem.memsize;
            O_bus_request.addr <= ex_mem.rs1data;
            O_bus_request.data <= ex_mem.rs2data;
            -- In case of a trap, record the memory address in MTVAL CSR
            csr_transfer.address_to_mtval <= ex_mem.rs1data;
        end if;
    end process;

    process (I_clk, I_areset, ex_mem) is
    begin
        if I_areset = '1' then
            mem_wb.rd <= (others => '0');
            mem_wb.rd_en <= '0';
            mem_wb.isload <= '0';
            mem_wb.rs1data <= all_zeros_c;
            mem_wb.alu_op <= alu_unknown;
            mem_wb.valid <= '0';
        elsif rising_edge(I_clk) then
            if I_sreset = '1' then
                mem_wb.rd <= (others => '0');
                mem_wb.rd_en <= '0';
                mem_wb.isload <= '0';
                mem_wb.rs1data <= all_zeros_c;
                mem_wb.alu_op <= alu_unknown;
                mem_wb.valid <= '0';
            else
                mem_wb.rd <= ex_mem.rd;
                mem_wb.isload <= ex_mem.isload;
                mem_wb.rs1data <= ex_mem.rs1data;
                mem_wb.alu_op <= ex_mem.alu_op;
                -- If there is a memory error...
                if I_bus_response.load_access_error = '1' or I_bus_response.load_misaligned_error = '1' or
                   I_bus_response.store_access_error = '1' or I_bus_response.store_misaligned_error = '1' then
                    -- Disable write-back
                    mem_wb.rd_en <= '0';
                    mem_wb.valid <= '0';
                else
                    mem_wb.rd_en <= ex_mem.rd_en;
                    mem_wb.valid <= ex_mem.valid;
                end if;
            end if;
        end if;
    end process;
    
    
    --
    -- MEM/WB stage: select data to write in the registers
    --
    process (I_bus_response, mem_wb) is
    begin
        -- Select correct part and perform zero/sign extension
        if mem_wb.alu_op = alu_lb then
            mem_wb.rddata(31 downto 8) <= (others => I_bus_response.data(7));
            mem_wb.rddata(7 downto 0) <= I_bus_response.data(7 downto 0);
        elsif mem_wb.alu_op = alu_lbu then
            mem_wb.rddata(31 downto 8) <= (others => '0');
            mem_wb.rddata(7 downto 0) <= I_bus_response.data(7 downto 0);
        elsif mem_wb.alu_op = alu_lh then
            mem_wb.rddata(31 downto 16) <= (others => I_bus_response.data(15));
            mem_wb.rddata(15 downto 0) <= I_bus_response.data(15 downto 0);
        elsif mem_wb.alu_op = alu_lhu then
            mem_wb.rddata(31 downto 15) <= (others => '0');
            mem_wb.rddata(15 downto 0) <= I_bus_response.data(15 downto 0);
        elsif mem_wb.alu_op = alu_lw then
            mem_wb.rddata <= I_bus_response.data;
        else
            mem_wb.rddata <= mem_wb.rs1data;
        end if;
    end process;
            
    --
    -- WB/BP stage
    --

    -- Save result if register file must be bypassed.
    -- When the registers are not write-through, this stage
    -- is needed.
    process (I_clk, I_areset) is
    begin
        if I_areset = '1' then
            wb_bp.rd <= (others => '0');
            wb_bp.rd_en <= '0';
            wb_bp.rddata <= all_zeros_c;
            wb_bp.valid <= '0';
        elsif rising_edge(I_clk) then
            if I_sreset = '1' then
                wb_bp.rd <= (others => '0');
                wb_bp.rd_en <= '0';
                wb_bp.rddata <= all_zeros_c;
                wb_bp.valid <= '0';
            else
                wb_bp.rd <= mem_wb.rd;
                wb_bp.rd_en <= mem_wb.rd_en;
                wb_bp.rddata <= mem_wb.rddata;
                wb_bp.valid <= mem_wb.valid;
            end if;
        end if;
    end process;



    --
    -- The MD unit, can be omitted by setting HAVE_MULDIV to false
    --
    
    muldivgen: if HAVE_MULDIV generate
        -- Multiplication Unit
        -- Check start of multiplication and load registers
        process (I_clk, I_areset, control, id_ex, ex_mem, mem_wb, wb_bp) is
        variable a_v, b_v : data_type;
        begin
            -- Check if forwarding result is needed
            if control.forwarda = "10" then
                a_v := ex_mem.rs1data;
            elsif control.forwarda = "01" then
                a_v := mem_wb.rddata;
            elsif control.forwarda = "11" then
                a_v := wb_bp.rddata;
            else
                a_v := id_ex.rs1data;
            end if;

            if control.forwardb = "10" then
                b_v := ex_mem.rs1data; -- !note: from register RS1
            elsif control.forwardb = "01" then
                b_v := mem_wb.rddata;
            elsif control.forwardb = "11" then
                b_v := wb_bp.rddata;
            else
                b_v := id_ex.rs2data;
            end if;
        
            if I_areset = '1' then
                md.rdata_a <= (others => '0');
                md.rdata_b <= (others => '0');
                md.mul_running <= '0';
            elsif rising_edge(I_clk) then
                if I_sreset = '1' then
                    md.rdata_a <= (others => '0');
                    md.rdata_b <= (others => '0');
                    md.mul_running <= '0';
                else
                    -- Clock in the multiplicand and multiplier
                    -- In the Cyclone V, these are embedded registers
                    -- in the DSP units.
                    if id_ex.md_start = '1' and control.trap_request = '0' and id_ex.md_op(2) = '0' then
                        if id_ex.md_op(1) = '1' then
                            if id_ex.md_op(0) = '1' then
                                md.rdata_a <= '0' & unsigned(a_v);
                            else
                                md.rdata_a <= a_v(31) & unsigned(a_v);
                            end if;
                            md.rdata_b <= '0' & unsigned(b_v);
                        else
                            md.rdata_a <= a_v(31) & unsigned(a_v);
                            md.rdata_b <= b_v(31) & unsigned(b_v);
                        end if;
                    end if;
                    -- Only start when start seen and multiply
                    md.mul_running <= id_ex.md_start and not control.trap_request and not id_ex.md_op(2);
                end if; -- sreset
            end if; -- posedge
        end process;

        -- Do the multiplication
        process(I_clk, I_areset) is
        begin
            if I_areset = '1' then
                md.mul_rd_int <= (others => '0');
            elsif rising_edge (I_clk) then
                if I_sreset = '1' then
                    md.mul_rd_int <= (others => '0');
                else
                    -- Do the multiplication and store in embedded registers
                    md.mul_rd_int <= signed(md.rdata_a) * signed(md.rdata_b);
                end if; -- sreset
            end if; -- posedge
        end process;
        md.mul_ready <= md.mul_running;

        -- Output multiplier result
        process (md, id_ex) is
        begin
            if id_ex.md_op(1) = '1' or id_ex.md_op(0) = '1' then
                md.mul <= std_logic_vector(md.mul_rd_int(63 downto 32));
            else
                md.mul <= std_logic_vector(md.mul_rd_int(31 downto 0));
            end if;
        end process;

        -- Division unit, retires one bit at a time
        process (I_clk, I_areset, control, id_ex, ex_mem, mem_wb, wb_bp) is
        variable a_v, b_v : data_type;
        variable div_running_v : std_logic;  
        variable count_v : integer range 0 to 32;
        begin
            -- Check if forwarding result is needed
            if control.forwarda = "10" then
                a_v := ex_mem.rs1data;
            elsif control.forwarda = "01" then
                a_v := mem_wb.rddata;
            elsif control.forwarda = "11" then
                a_v := wb_bp.rddata;
            else
                a_v := id_ex.rs1data;
            end if;

            if control.forwardb = "10" then
                b_v := ex_mem.rs1data; -- !note: from register RS1
            elsif control.forwardb = "01" then
                b_v := mem_wb.rddata;
            elsif control.forwardb = "11" then
                b_v := wb_bp.rddata;
            else
                b_v := id_ex.rs2data;
            end if;
            
            if I_areset = '1' then
                -- Reset everything
                count_v := 0;
                md_buf1 <= (others => '0');
                md_buf2 <= (others => '0');
                md.divisor <= (others => '0');
                div_running_v := '0';
                md.div_ready <= '0';
                md.outsign <= '0';
            elsif rising_edge(I_clk) then 
                if I_sreset = '1' then
                    -- Reset everything
                    count_v := 0;
                    md_buf1 <= (others => '0');
                    md_buf2 <= (others => '0');
                    md.divisor <= (others => '0');
                    div_running_v := '0';
                    md.div_ready <= '0';
                    md.outsign <= '0';
                else
                    -- If start and dividing...
                    md.div_ready <= '0';
                    if id_ex.md_start = '1' and id_ex.md_op(2) = '1' and control.trap_request = '0' then
                        div_running_v := '1';
                        count_v := 0;
                    end if;
                    if div_running_v = '1' then
                        case count_v is 
                            when 0 => 
                                md_buf1 <= (others => '0');
                                -- If signed divide, check for negative
                                -- value and make it positive
                                if id_ex.md_op(0) = '0' and a_v(31) = '1' then
                                    md_buf2 <= unsigned(not a_v) + 1;
                                else
                                    md_buf2 <= unsigned(a_v);
                                end if;
                                if id_ex.md_op(0) = '0' and b_v(31) = '1' then
                                    md.divisor <= unsigned(not b_v) + 1;
                                else
                                    md.divisor <= unsigned(b_v); 
                                end if;
                                count_v := 1; 
                                md.div_ready <= '0';
                                -- Determine the result sign
                                if (id_ex.md_op(0) = '0' and id_ex.md_op(1) = '0' and (a_v(31) /= b_v(31)) and b_v /= all_zeros_c) or (id_ex.md_op(0) = '0' and id_ex.md_op(1) = '1' and a_v(31) = '1') then
                                    md.outsign <= '1';
                                else
                                    md.outsign <= '0';
                                end if;

                            when others =>
                                -- Do the division
                                if md.buf(62 downto 31) >= md.divisor then 
                                    md_buf1 <= '0' & (md.buf(61 downto 31) - md.divisor(30 downto 0)); 
                                    md_buf2 <= md_buf2(30 downto 0) & '1'; 
                                else 
                                    md.buf <= md.buf(62 downto 0) & '0'; 
                                end if;
                                -- Do this 32 times, last one outputs the result
                                if count_v /= 32 then 
                                    -- Signal ready one clock before
                                    if count_v = 31 then
                                        md.div_ready <= '1';
                                    end if;
                                    count_v := count_v + 1;
                                else
                                    -- Signal ready
                                    count_v := 0;
                                    div_running_v := '0';
                                end if; 
                        end case; 
                    end if;
                end if; -- sreset
            end if; -- posedge
-- synthesis translate_off
            -- Only to view in simulator
            md.count <= count_v;
-- synthesis translate_on
        end process;

        -- Select the correct signedness of the results
        process (md.outsign, md_buf2, md_buf1) is
        begin
            if md.outsign = '1' then
                md.quotient <= not md_buf2 + 1;
                md.remainder <= not md_buf1 + 1;
            else
                md.quotient <= md_buf2;
                md.remainder <= md_buf1; 
            end if;
        end process;

        -- Select the divider output
        md.div <= std_logic_vector(md.remainder) when id_ex.md_op(1) = '1' else std_logic_vector(md.quotient);
        
        -- Signal that we are ready
        md.ready <= md.div_ready or md.mul_ready;
        
    end generate; -- generate MD unit

    -- If we don't have an MD unit, set some signals
    -- to default values. The synthesizer will remove the hardware.
    muldivgennot: if not HAVE_MULDIV generate
        md.ready <= '0';
        md.mul <= (others => '0');
        md.div <= (others => '0');
    end generate;    




    --
    -- CSR
    --

    -- Select the correct info
    csr_access.op <= id_ex.csr_op;
    -- Set access parameters
    csr_access.address <= id_ex.csr_addr;
    csr_access.immrs1 <= id_ex.csr_immrs1;

    -- Forward data to CSR hardware
    process (control, id_ex, ex_mem, mem_wb, wb_bp) is
    begin
        if control.forwarda = "10" then
            csr_access.data_to_csr <= ex_mem.rs1data;
        elsif control.forwarda = "01" then
            csr_access.data_to_csr <= mem_wb.rddata;
        elsif control.forwarda = "11" then
            csr_access.data_to_csr <= wb_bp.rddata;
        else
            csr_access.data_to_csr <= id_ex.rs1data;
        end if;
    end process;    

    -- CSR hardware
    process (I_clk, I_areset, csr_reg, csr_access, control, I_dm_core_data_request) is
    variable csr_addr_v : integer range 0 to csr_size-1;
    variable csr_content_v : data_type;
    begin
        -- Fetch CSR address
        if control.stall_on_debug = '1' then
            csr_addr_v := to_integer(unsigned(I_dm_core_data_request.address(11 downto 0)));
        else
            csr_addr_v := to_integer(unsigned(csr_access.address));
        end if;

        -- Check for correct access
        if control.stall_on_debug = '0' and csr_access.op = csr_nop then
            control.illegal_instruction_csr <= '0';
        elsif control.stall_on_debug = '0' and csr_access.address(11 downto 10) = "11" and (csr_access.op = csr_rw or csr_access.op = csr_rwi or csr_access.immrs1 /= "00000") then
            control.illegal_instruction_csr <= '1';
        elsif control.stall_on_debug = '1' and ((I_dm_core_data_request.writecsr = '0' and I_dm_core_data_request.readcsr = '0') or OCD_CSR_CHECK_DISABLE) then
            control.illegal_instruction_csr <= '0';
        elsif csr_addr_v = cycle_addr or
              csr_addr_v = time_addr or
              csr_addr_v = instret_addr or
              csr_addr_v = cycleh_addr or
              csr_addr_v = timeh_addr or
              csr_addr_v = instreth_addr or

              csr_addr_v = mvendorid_addr or
              csr_addr_v = marchid_addr or
              csr_addr_v = mimpid_addr or
              csr_addr_v = mhartid_addr or
              csr_addr_v = mconfigptr_addr or
              csr_addr_v = mstatus_addr or
              csr_addr_v = mstatush_addr or
              csr_addr_v = misa_addr or
              
              csr_addr_v = mie_addr or
              csr_addr_v = mtvec_addr or
              csr_addr_v = mstatush_addr or
              csr_addr_v = mcountinhibit_addr or
              csr_addr_v = mscratch_addr or
              csr_addr_v = mepc_addr or
              csr_addr_v = mcause_addr or
              csr_addr_v = mtval_addr or
              csr_addr_v = mip_addr or
              
              csr_addr_v = mcycle_addr or
              csr_addr_v = minstret_addr or
              csr_addr_v = mcycleh_addr or
              csr_addr_v = minstreth_addr or
             
             (csr_addr_v = dcsr_addr and control.stall_on_debug = '1' and HAVE_OCD) or
             (csr_addr_v = dpc_addr and control.stall_on_debug = '1' and HAVE_OCD) or
             (csr_addr_v = tselect_addr and HAVE_OCD) or
             (csr_addr_v = tdata1_addr and HAVE_OCD) or
             (csr_addr_v = tdata2_addr and HAVE_OCD) or
             (csr_addr_v = tdata3_addr and HAVE_OCD) or
             (csr_addr_v = tinfo_addr and HAVE_OCD) or

              csr_addr_v = mxhw_addr or
              csr_addr_v = mxspeed_addr then
            control.illegal_instruction_csr <= '0';
        else 
            control.illegal_instruction_csr <= '1';
        end if;


        -- Output to ALU, unused CSRs return all zero bits
        case csr_addr_v is
            -- Basic user
            when cycle_addr         => csr_access.data_from_csr <= csr_reg.mcycle;
            when time_addr          => csr_access.data_from_csr <= csr_reg.mtime;
            when instret_addr       => csr_access.data_from_csr <= csr_reg.minstret;
            when cycleh_addr        => csr_access.data_from_csr <= csr_reg.mcycleh;
            when timeh_addr         => csr_access.data_from_csr <= csr_reg.mtimeh;
            when instreth_addr      => csr_access.data_from_csr <= csr_reg.minstreth;
            -- Basic M
            when mvendorid_addr     => csr_access.data_from_csr <= csr_reg.mvendorid;
            when marchid_addr       => csr_access.data_from_csr <= csr_reg.marchid;
            when mimpid_addr        => csr_access.data_from_csr <= csr_reg.mimpid;
            when mhartid_addr       => csr_access.data_from_csr <= csr_reg.mhartid;
            when misa_addr          => csr_access.data_from_csr <= csr_reg.misa;
            when mcycle_addr        => csr_access.data_from_csr <= csr_reg.mcycle;
            when minstret_addr      => csr_access.data_from_csr <= csr_reg.minstret;
            when mcycleh_addr       => csr_access.data_from_csr <= csr_reg.mcycleh;
            when minstreth_addr     => csr_access.data_from_csr <= csr_reg.minstreth;
            when mcountinhibit_addr => csr_access.data_from_csr <= csr_reg.mcountinhibit;
            when mstatus_addr       => csr_access.data_from_csr <= csr_reg.mstatus;
            -- Trap related
            when mtvec_addr         => csr_access.data_from_csr <= csr_reg.mtvec;
            when mepc_addr          => csr_access.data_from_csr <= csr_reg.mepc;
            when mscratch_addr      => csr_access.data_from_csr <= csr_reg.mscratch;
            when mtval_addr         => csr_access.data_from_csr <= csr_reg.mtval;
            when mie_addr           => csr_access.data_from_csr <= csr_reg.mie;
            when mip_addr           => csr_access.data_from_csr <= csr_reg.mip;
            when mcause_addr        => csr_access.data_from_csr <= csr_reg.mcause;
            -- Debug
            when dcsr_addr          => csr_access.data_from_csr <= csr_reg.dcsr;
            when dpc_addr           => csr_access.data_from_csr <= csr_reg.dpc;
            when tselect_addr       => csr_access.data_from_csr <= csr_reg.tselect;
            when tdata1_addr        => csr_access.data_from_csr <= csr_reg.tdata1;
            when tdata2_addr        => csr_access.data_from_csr <= csr_reg.tdata2;
            when tinfo_addr         => csr_access.data_from_csr <= csr_reg.tinfo;
            -- Custom read-only
            when mxhw_addr          => csr_access.data_from_csr <= csr_reg.mxhw;
            when mxspeed_addr       => csr_access.data_from_csr <= csr_reg.mxspeed;
            when others             => csr_access.data_from_csr <= (others => '0');
        end case;

        if I_areset = '1' then
            csr_reg.mcycle <= all_zeros_c;
            csr_reg.mcycleh <= all_zeros_c;
            csr_reg.minstret <= all_zeros_c;
            csr_reg.minstreth <= all_zeros_c;
            csr_reg.mcountinhibit <= all_zeros_c;
            csr_reg.mtvec <= all_zeros_c;
            csr_reg.mepc <= all_zeros_c;
            csr_reg.mscratch <= all_zeros_c;
            csr_reg.mtval <= all_zeros_c;
            csr_reg.mie <= all_zeros_c;
            csr_reg.mcause <= all_zeros_c;
            csr_reg.mstatus <= all_zeros_c;
            control.nmi_lockout <= '0';
            csr_reg.dcsr <= (others => '0');
            csr_reg.dpc <= (others => '0');
            csr_reg.tdata1 <= (others => '0');
            csr_reg.tdata2 <= (others => '0');
        elsif rising_edge(I_clk) then
            if I_sreset = '1' then
                csr_reg.mcycle <= all_zeros_c;
                csr_reg.mcycleh <= all_zeros_c;
                csr_reg.minstret <= all_zeros_c;
                csr_reg.minstreth <= all_zeros_c;
                csr_reg.mcountinhibit <= all_zeros_c;
                csr_reg.mtvec <= all_zeros_c;
                csr_reg.mepc <= all_zeros_c;
                csr_reg.mscratch <= all_zeros_c;
                csr_reg.mtval <= all_zeros_c;
                csr_reg.mie <= all_zeros_c;
                csr_reg.mcause <= all_zeros_c;
                csr_reg.mstatus <= all_zeros_c;
                control.nmi_lockout <= '0';
                csr_reg.dcsr <= (others => '0');
                csr_reg.dpc <= (others => '0');
                csr_reg.tdata1 <= (others => '0');
                csr_reg.tdata2 <= (others => '0');
            else
                -- Update the cycle counter
                if csr_reg.mcountinhibit(0) = '0' then
                    csr_reg.mcycle <= std_logic_vector(unsigned(csr_reg.mcycle) + 1);
                    if csr_reg.mcycle = all_ones_c then
                        csr_reg.mcycleh <= std_logic_vector(unsigned(csr_reg.mcycleh) + 1);
                    end if;
                end if;
                -- Update the instruction retired counter
                if csr_reg.mcountinhibit(2) = '0' then
                    if control.instret = '1' then
                        csr_reg.minstret <= std_logic_vector(unsigned(csr_reg.minstret) + 1);
                        if csr_reg.minstret = all_ones_c then
                            csr_reg.minstreth <= std_logic_vector(unsigned(csr_reg.minstreth) + 1);
                        end if;
                    end if;
                end if;

                -- Update the CSR. Follows the RISC-V unpriv spec, 5.1.1. CSR Instructions
                if csr_access.op /= csr_nop and control.trap_request = '0' and control.stall_on_debug = '0' then
                    -- Select the CSR
                    case csr_addr_v is
                        when mcycle_addr        => csr_content_v := csr_reg.mcycle;
                        when mcycleh_addr       => csr_content_v := csr_reg.mcycleh;
                        when minstret_addr      => csr_content_v := csr_reg.minstret;
                        when minstreth_addr     => csr_content_v := csr_reg.minstreth;
                        when mcountinhibit_addr => csr_content_v := csr_reg.mcountinhibit;
                        when mtvec_addr         => csr_content_v := csr_reg.mtvec;
                        when mepc_addr          => csr_content_v := csr_reg.mepc;
                        when mscratch_addr      => csr_content_v := csr_reg.mscratch;
                        when mtval_addr         => csr_content_v := csr_reg.mtval;
                        when mie_addr           => csr_content_v := csr_reg.mie;
                        when mcause_addr        => csr_content_v := csr_reg.mcause;
                        when mstatus_addr       => csr_content_v := csr_reg.mstatus;
                        when others             => csr_content_v := (others => '-');
                    end case;
                    -- Do the operation
                    -- Some bits should be ignored or hard wired to 0
                    -- but we just ignore them
                    case csr_access.op is
                        when csr_rw =>
                            csr_content_v := csr_access.data_to_csr;
                        when csr_rs =>
                            csr_content_v := csr_content_v or csr_access.data_to_csr;
                        when csr_rc =>
                            csr_content_v := csr_content_v and not csr_access.data_to_csr;
                        when csr_rwi =>
                            csr_content_v(csr_content_v'left downto 5) := (others => '0');
                            csr_content_v(4 downto 0) := csr_access.immrs1;
                        when csr_rsi =>
                            csr_content_v(4 downto 0) := csr_content_v(4 downto 0) or csr_access.immrs1(4 downto 0);
                        when csr_rci =>
                            csr_content_v(4 downto 0) := csr_content_v(4 downto 0) and not csr_access.immrs1(4 downto 0);
                        when others =>
                            null;
                    end case;
                    -- Write back
                    case csr_addr_v is
                        when mcycle_addr        => csr_reg.mcycle <= csr_content_v;
                        when mcycleh_addr       => csr_reg.mcycleh <= csr_content_v;
                        when minstret_addr      => csr_reg.minstret <= csr_content_v;
                        when minstreth_addr     => csr_reg.minstreth <= csr_content_v;
                        when mcountinhibit_addr => csr_reg.mcountinhibit <= csr_content_v;
                        when mtvec_addr         => csr_reg.mtvec <= csr_content_v;
                        when mepc_addr          => csr_reg.mepc <= csr_content_v;
                        when mscratch_addr      => csr_reg.mscratch <= csr_content_v;
                        when mtval_addr         => csr_reg.mtval <= csr_content_v;
                        when mie_addr           => csr_reg.mie <= csr_content_v;
                        when mcause_addr        => csr_reg.mcause <= csr_content_v;
                        when mstatus_addr       => csr_reg.mstatus <= csr_content_v;
                        when others => null;
                    end case;
                    
                end if;

                -- Set the cause for halting in DCSR
                -- cause(3) is load flag
                if csr_reg.dcsr_cause(3) = '1' and HAVE_OCD then
                    csr_reg.dcsr(8 downto 6) <= csr_reg.dcsr_cause(2 downto 0);
                    if csr_reg.dcsr_cause(2 downto 0) = "010" then
                        csr_reg.tdata1(22) <= '1';  -- hit0
                    end if;
                end if;
                -- If a halt/break/step, load DPC with PC
                if control.load_dpc = '1' and HAVE_OCD then
                    csr_reg.dpc <= ex_mem.pc;
                end if;

                -- When we're in debug mode ...
                if control.stall_on_debug = '1' and I_dm_core_data_request.writecsr = '1' and HAVE_OCD then
                    case csr_addr_v is
                        when mcycle_addr => csr_reg.mcycle <= I_dm_core_data_request.data;
                        when mcycleh_addr => csr_reg.mcycleh <= I_dm_core_data_request.data;
                        when minstret_addr => csr_reg.minstret <= I_dm_core_data_request.data;
                        when minstreth_addr => csr_reg.minstreth <= I_dm_core_data_request.data;
                        when mstatus_addr => csr_reg.mstatus <= I_dm_core_data_request.data;
                        when mie_addr => csr_reg.mie <= I_dm_core_data_request.data;
                        when mtvec_addr => csr_reg.mtvec <= I_dm_core_data_request.data;
                        when mcountinhibit_addr => csr_reg.mcountinhibit <= I_dm_core_data_request.data;
                        when mscratch_addr => csr_reg.mscratch <= I_dm_core_data_request.data;
                        when mepc_addr => csr_reg.mepc <= I_dm_core_data_request.data;
                        when mcause_addr => csr_reg.mcause <= I_dm_core_data_request.data;
                        when mtval_addr => csr_reg.mtval <= I_dm_core_data_request.data;
                        when dcsr_addr => csr_reg.dcsr <= I_dm_core_data_request.data;
                        when dpc_addr => csr_reg.dpc <= I_dm_core_data_request.data;
                        when tdata1_addr => csr_reg.tdata1 <= I_dm_core_data_request.data;
                        when tdata2_addr => csr_reg.tdata2 <= I_dm_core_data_request.data;
                        when others => null;
                    end case;
                end if;
                
                -- Zero out unused bits
                csr_reg.mcountinhibit(31 downto 3) <= (others => '0');
                csr_reg.mcountinhibit(1) <= '0';

                -- Set all bits hard to 0 except MTIE (7), MSIE (3)
                csr_reg.mie(csr_reg.mie'left downto 8) <= (others => '0');
                csr_reg.mie(6 downto 4) <= (others => '0');
                csr_reg.mie(2 downto 0) <= (others => '0');
                
                -- MCAUSE doesn't use that many bits...
                -- Only Interrupt Bit and 5 LSB are needed
                csr_reg.mcause(30 downto 5) <= (others => '0');

                -- Set most bits of mstatus to 0
                csr_reg.mstatus(csr_reg.mstatus'left downto 13) <= (others => '0');
                csr_reg.mstatus(10 downto 8) <= (others => '0');
                csr_reg.mstatus(4 downto 4) <= (others => '0');
                csr_reg.mstatus(2 downto 0) <= (others => '0');
                
                if HAVE_OCD then
                    -- Debug registers
                    csr_reg.dcsr(1 downto 0) <= "11";                -- Alwyas M-mode
                    csr_reg.dcsr(3) <= '0';                          -- NMI interrupt pending not used
                    csr_reg.dcsr(4) <= '0';                          -- mpriven not used
                    csr_reg.dcsr(5) <= '0';                          -- v not used
                    csr_reg.dcsr(9) <= '0';                          -- stoptime not used
                    csr_reg.dcsr(10) <= '0';                         -- stopcount not used
                    csr_reg.dcsr(11) <= '0';                         -- stepie not used
                    csr_reg.dcsr(12) <= '0';                         -- ebreaku not used
                    csr_reg.dcsr(13) <= '0';                         -- ebreaks not used
                    csr_reg.dcsr(14) <= '0';                         -- reserved
                    csr_reg.dcsr(16) <= '0';                         -- ebreakvu not used
                    csr_reg.dcsr(17) <= '0';                         -- ebreakvs not used
                    csr_reg.dcsr(27 downto 18) <= (others => '0');   -- reserved
                    csr_reg.dcsr(31 downto 28) <= "0100";            -- version 1.0

                    csr_reg.tdata1(31 downto 28) <= x"6";            -- always type 6 (mcontrol6)
                    csr_reg.tdata1(26) <= '0';                       -- uncertain
                    csr_reg.tdata1(25) <= '0';                       -- hit1 = 0
                    csr_reg.tdata1(24) <= '0';                       -- vs
                    csr_reg.tdata1(23) <= '0';                       -- vu
                    csr_reg.tdata1(20) <= '0';                       -- 0
                    csr_reg.tdata1(19) <= '0';                       -- 0
                    csr_reg.tdata1(11) <= '0';                       -- chain
                    csr_reg.tdata1(5) <= '0';                        -- uncertainen
                    csr_reg.tdata1(4) <= '0';                        -- S mode
                    csr_reg.tdata1(3) <= '0';                        -- U mode
                    csr_reg.tdata1(1) <= '0';                        -- no store
                    csr_reg.tdata1(0) <= '0';                        -- no load

                    csr_reg.dpc(1 downto 0) <= "00";                 -- LSB always 0, since IALIGN = 32
                else
                    csr_reg.dcsr <= (others => '0');
                    csr_reg.dpc <= (others => '0');
                    csr_reg.tdata1 <= (others => '0');
                    csr_reg.tdata2 <= (others => '0');
                end if;

                -- Interrupt handling takes priority over possible user
                -- update of the CSRs.
                -- The LIC checks if exceptions/interrupts are enabled.
                if control.trap_request = '1' and control.stall_on_debug = '0' then
                    -- Copy mie to mpie
                    csr_reg.mstatus(7) <= csr_reg.mstatus(3);
                    -- Set M mode
                    csr_reg.mstatus(12 downto 11) <= "11";
                    -- Disable interrupts
                    csr_reg.mstatus(3) <= '0';
                    -- Copy mcause
                    csr_reg.mcause <= control.trap_mcause;
                    -- Save PC at the point of interrupt, note that a
                    -- memory fault occurs when the memory operation
                    -- is in the EX_MEM stage, so we must get the PC
                    -- of that memory operation. Otherwise the fault
                    -- occurs is the ID_EX stage.
                    if control.trap_memfault = '1' then
                        csr_reg.mepc <= ex_mem.pc;
                    else
                        csr_reg.mepc <= id_ex.pc;
                    end if;
                    -- Set MTVAL, see priv ISA, S.2.1.1.16
                    if control.trap_mcause = x"00000000" then
                        -- Instruction misaligned fault, set MTVAL to all zeros
                        csr_reg.mtval <= (others => '0');
                    elsif control.trap_mcause = x"00000001" then
                        -- Instruction access fault, set MTVAL to all zeros
                        csr_reg.mtval <= (others => '0');
                    elsif control.trap_mcause = x"00000002" then
                        -- Illegal instruction fault, set MTVAL to all zeros
                        csr_reg.mtval <= (others => '0');
                    elsif control.trap_mcause = x"00000003" then
                        -- Breakpoint, set MTVAL to all zeros
                        csr_reg.mtval <= (others => '0');
                    else
                        -- Latch address from address bus
                        csr_reg.mtval <= csr_transfer.address_to_mtval;
                    end if;
                    -- Lock out further NMI interrupts
                    if I_intrio(31) = '1' then
                        control.nmi_lockout <= '1';
                    end if;
                elsif control.trap_release = '1' then
                    -- Copy mpie to mie
                    csr_reg.mstatus(3) <= csr_reg.mstatus(7);
                    -- ??
                    csr_reg.mstatus(7) <= '1';
                    -- Keep M mode
                    csr_reg.mstatus(12 downto 11) <= "11";
                 -- Enable further NMI interrupts
                    control.nmi_lockout <= '0';
                end if;
                
            end if; -- sreset
            
        end if; -- rising_edge

        -- Calculate the MTVEC to be loaded in the PC on trap
        if VECTORED_MTVEC and csr_reg.mtvec(0) = '1' and csr_reg.mcause(31) = '1' then
            csr_transfer.mtvec_to_pc <= std_logic_vector(unsigned(csr_reg.mtvec(csr_reg.mtvec'left downto 2)) + unsigned(csr_reg.mcause(5 downto 0))) & "00";
        else
            csr_transfer.mtvec_to_pc <= csr_reg.mtvec(csr_reg.mtvec'left downto 2) & "00";
        end if;

        -- Lowest two bits of mepc always 0, see priv ISA, S.2.1.1.14
        csr_reg.mepc(1 downto 0) <= "00";

    end process;

    -- Transfer of MEPC to PC
    csr_transfer.mepc_to_pc <= csr_reg.mepc;
    -- Transfer of DPC to PC
    csr_transfer.dpc_to_pc <= csr_reg.dpc;
    
    -- Hard wired CSR's
    csr_reg.mvendorid <= (others => '0'); --
    csr_reg.marchid <= (others => '0');
    csr_reg.mimpid <= std_logic_vector(to_unsigned(HW_VERSION, 32));
    csr_reg.mhartid <= (others => '0');
    csr_reg.misa(31 downto 13) <= x"4000" & "000";
    csr_reg.misa(12) <= '1' when HAVE_MULDIV else '0';
    csr_reg.misa(11 downto 4) <= x"10" when NUMBER_OF_REGISTERS = 32 else x"01";
    csr_reg.misa(3 downto 0) <= x"0"; --x"2" when HAVE_ZBA and HAVE_ZBB and HAVE_ZBS else x"0";

    -- Custom read-only hardware description
    csr_reg.mxhw(00) <= '1'; -- GPIOA, always present
    csr_reg.mxhw(01) <= '0'; -- reserved
    csr_reg.mxhw(02) <= '0'; --boolean_to_std_logic(HAVE_ZBKB);
    csr_reg.mxhw(03) <= '0'; --boolean_to_std_logic(FAST_MEM);
    csr_reg.mxhw(04) <= boolean_to_std_logic(HAVE_UART1);
    csr_reg.mxhw(05) <= boolean_to_std_logic(HAVE_UART2);
    csr_reg.mxhw(06) <= boolean_to_std_logic(HAVE_I2C1);
    csr_reg.mxhw(07) <= boolean_to_std_logic(HAVE_I2C2);
    csr_reg.mxhw(08) <= boolean_to_std_logic(HAVE_SPI1);
    csr_reg.mxhw(09) <= boolean_to_std_logic(HAVE_SPI2);
    csr_reg.mxhw(10) <= boolean_to_std_logic(HAVE_TIMER1);
    csr_reg.mxhw(11) <= boolean_to_std_logic(HAVE_TIMER2);
    csr_reg.mxhw(12) <= '0'; -- reserved
    csr_reg.mxhw(13) <= '0'; -- reserved
    csr_reg.mxhw(14) <= '0'; -- reserved
    csr_reg.mxhw(15) <= '1'; -- TIME/TIMEH, always present
    csr_reg.mxhw(16) <= boolean_to_std_logic(HAVE_MULDIV);
    csr_reg.mxhw(17) <= '0'; --boolean_to_std_logic(FAST_DIVIDE and HAVE_MULDIV);
    csr_reg.mxhw(18) <= boolean_to_std_logic(HAVE_BOOTLOADER_ROM);
    csr_reg.mxhw(19) <= '1'; -- not avail: boolean_to_std_logic(HAVE_REGISTERS_IN_RAM);
    csr_reg.mxhw(20) <= boolean_to_std_logic(HAVE_ZBA);
    csr_reg.mxhw(21) <= '0'; -- boolean_to_std_logic(HAVE_ZIMOP);
    csr_reg.mxhw(22) <= boolean_to_std_logic(HAVE_ZICOND);
    csr_reg.mxhw(23) <= boolean_to_std_logic(HAVE_ZBS);
    csr_reg.mxhw(24) <= boolean_to_std_logic(UART1_BREAK_RESETS);
    csr_reg.mxhw(25) <= boolean_to_std_logic(HAVE_WDT);
    csr_reg.mxhw(26) <= '0'; -- boolean_to_std_logic(HAVE_ZIHPM);
    csr_reg.mxhw(27) <= boolean_to_std_logic(HAVE_OCD);
    csr_reg.mxhw(28) <= boolean_to_std_logic(HAVE_MSI);
    csr_reg.mxhw(29) <= '0'; -- not avail. boolean_to_std_logic(BUFFER_IO_RESPONSE);
    csr_reg.mxhw(30) <= '0'; -- boolean_to_std_logic(HAVE_ZBB);
    csr_reg.mxhw(31) <= boolean_to_std_logic(HAVE_CRC);

    -- Custom read-only synthesized clock frequency
    csr_reg.mxspeed <= std_logic_vector(to_unsigned(SYSTEM_FREQUENCY, 32));

    -- Copy system timer info
    csr_reg.mtime <= I_mtime;
    csr_reg.mtimeh <= I_mtimeh;

    -- MIP is copy of the I/O interrupts
    csr_reg.mip <= I_intrio;

    -- Debug tinfo
    csr_reg.tinfo <= x"01000040" when HAVE_OCD else (others => '0'); -- v1, only mcontrol6

    -- Only 1 hw breakpoint
    csr_reg.tselect <= (others => '0');



    --
    -- Local interrupt controller
    --

    -- The Local Interrupt Controller (LIC) determines which
    -- trap is to be served. Note that interrupts will only
    -- be served if the processor is in the exec and wfi states.
    -- Exceptions will be served in the exec and mem states,
    process (I_intrio, I_bus_response,
             I_instr_response.instr_access_error, control, csr_reg) is
    begin
        control.trap_request <= '0';
        control.trap_release <= '0';
        control.trap_mcause <= (others => '0');
        control.trap_memfault <= '0';
        
        -- Priority as of Table 3.7 of "Volume II: RISC-V Privileged Architectures V20211203"
        -- Local hardware interrupts take priority over exceptions, the RISC-V system timer
        -- has the lowest hardware interrupt priority. Not all exceptions are implemented.
        -- Interrupts only when not stepping.
        
        -- NOTE: load/store misaligned/access before other, because they are the oldest
        -- instructions in the pipeline
        
        -- NMI triggered by watchdog timeout, cannot be blocked, except by stepping.
        if I_intrio(31) = '1' and control.may_interrupt ='1' and control.nmi_lockout = '0' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(31, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- Currently unassigned
        elsif I_intrio(30) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(30, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- Currently unassigned
        elsif I_intrio(29) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(29, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- Currently unassigned
        elsif I_intrio(28) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(28, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- SPI1
        elsif I_intrio(27) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(27, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- I2C1
        elsif I_intrio(26) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(26, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- SPI2
        elsif I_intrio(25) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(25, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- I2C2
        elsif I_intrio(24) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(24, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- UART1
        elsif I_intrio(23) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(23, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- Currently unassigned
        elsif I_intrio(22) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(22, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- TIMER2
        elsif I_intrio(21) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1'and control.isstepping = '0'  then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(21, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- TIMER1
        elsif I_intrio(20) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(20, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- Currently unassigned
        elsif I_intrio(19) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(19, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- EXTI
        elsif I_intrio(18) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(18, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- Currently unassigned
        elsif I_intrio(17) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(17, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- Currently unassigned
        elsif I_intrio(16) = '1' and csr_reg.mstatus(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(16, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- RISC-V machine software interrupt
        elsif I_intrio(3) = '1' and csr_reg.mstatus(3) = '1' and csr_reg.mie(3) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(3, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- RISC-V external timer interrupt
        elsif I_intrio(7) = '1' and csr_reg.mstatus(3) = '1' and csr_reg.mie(7) = '1' and control.may_interrupt ='1' and control.isstepping = '0' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(7, control.trap_mcause'length));
            control.trap_mcause(31) <= '1';
        -- Exceptions from here. Can always start a trap.
        -- Load access error (unimplemented memory)
        elsif I_bus_response.load_access_error = '1' then
            control.trap_request <= '1';
            control.trap_memfault <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(5, control.trap_mcause'length));
        -- Store access error (unimplemented memory)
        elsif I_bus_response.store_access_error = '1' then
            control.trap_request <= '1';
            control.trap_memfault <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(7, control.trap_mcause'length));
        -- Load misaligned
        elsif I_bus_response.load_misaligned_error = '1' then
            control.trap_request <= '1';
            control.trap_memfault <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(4, control.trap_mcause'length));
        -- Store misaligned
        elsif I_bus_response.store_misaligned_error = '1' then
            control.trap_request <= '1';
            control.trap_memfault <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(6, control.trap_mcause'length));
        -- Instruction access from unimplemented ROM
        elsif control.instr_access_error(1) = '1' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(1, control.trap_mcause'length));
        -- Illegal instruction, can also be a CSR instruction problem
        elsif control.illegal_instruction_decode = '1' or control.illegal_instruction_csr = '1' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(2, control.trap_mcause'length));
        -- Instruction misaligned
        elsif control.instruction_misaligned = '1' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(0, control.trap_mcause'length));
        -- ECALL instruction
        elsif control.ecall_request = '1' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(11, control.trap_mcause'length));
        -- EBREAK instruction, Priv Spec if dcsr.EBREAKM is off
        elsif control.ebreak_request = '1' then
            control.trap_request <= '1';
            control.trap_mcause <= std_logic_vector(to_unsigned(3, control.trap_mcause'length));
        end if;

        -- Signal interrupt release
        if control.mret_request_delay = '1' then
            control.trap_release <= '1';
        end if;
    end process;


    --
    -- Communication to and from DM
    --
    
    -- These assignments regulate the data transfer to and from the Debug Module (DM)
    -- Data response from core to DM
    O_dm_core_data_response.data <= csr_access.data_from_csr when I_dm_core_data_request.readcsr = '1' else
                                    data_from_gpr when I_dm_core_data_request.readgpr = '1' else
                                    I_bus_response.data when I_dm_core_data_request.readmem = '1' else
                                    x"00000000";

    -- Send an ACK if there is a ready from memory OR we're in debug and there is a data memory error
    O_dm_core_data_response.ack <= I_bus_response.ready or 
                                  (control.indebug and 
                                  (I_bus_response.load_misaligned_error or I_bus_response.store_misaligned_error or
                                   I_bus_response.load_access_error or I_bus_response.store_access_error));

    -- Send bus error if there is a data memory error
    O_dm_core_data_response.buserr <= control.indebug and
                                     (I_bus_response.load_misaligned_error or I_bus_response.store_misaligned_error or
                                      I_bus_response.load_access_error or I_bus_response.store_access_error);

    -- Send an exception if there is an illegal instruction executed
    O_dm_core_data_response.excep <= control.indebug and
                                    (control.illegal_instruction_csr or control.illegal_instruction_decode);



    
-- synthesis translate_off
    -- This process waits for a rising edge of the clock
    -- and then outputs the time, the PC in de EX stage, the
    -- instruction, the state of the controller and the ALU
    -- operation to the file `output.txt`.
    simgen: if SIMULATION_EXTRA generate
        process is
        file outfile : text open write_mode is "output.txt";
        variable line_buf : line;
        begin
            -- Wait for rising edge and write time
            wait until I_clk = '1';
            write(line_buf, string'("TI = "));
            write(line_buf, now, right, 10);
            -- Needed to stabilize signals
            wait for 2 ns;
            -- Write the rest
            write(line_buf, string'(", PC = "));
            hwrite(line_buf, id_ex.pc);
            write(line_buf, string'(", IN = "));
            hwrite(line_buf, id_ex.instr);
            write(line_buf, string'(", ST = "));
            write(line_buf, state_type'image(control.state), left, 12);
            write(line_buf, string'(", OP = "));
            write(line_buf, alu_op_type'image(id_ex.alu_op));
            writeline(outfile, line_buf);
        end process;
    end generate;
-- synthesis translate_on

end architecture rtl;
