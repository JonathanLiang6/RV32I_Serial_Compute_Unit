`timescale 1ns / 1ps 

module sccomp(
    input clk,                // 全局时钟输入
    input rstn,               // 全局复位输入（低电平有效）
    input [15:0] sw_i,        // 开关输入信号，用于控制功能
    output [7:0] disp_seg_o,  // 数码管段选输出
    output [7:0] disp_an_o    // 数码管位选输出
);

// 时钟分频寄存器，用于生成不同频率的时钟信号
reg [31:0] clkdiv;          
wire Clk_CPU;              

// 时钟分频逻辑：在上升沿时钟或下降沿复位信号时更新分频寄存器
always @(posedge clk or negedge rstn) begin
    if (!rstn)               // 如果复位信号有效（低电平），将分频寄存器清零
        clkdiv <= 0;
    else                     // 否则，分频寄存器递增
        clkdiv <= clkdiv + 1'b1;
end

// CPU 时钟选择逻辑：
// 如果 sw_i[15] 为高电平，则选择较慢的 clkdiv[27] 作为 CPU 时钟；否则选择较快的 clkdiv[25]
assign Clk_CPU = (sw_i[15]) ? clkdiv[27] : clkdiv[25];
// 结合 sw_i[1] 信号进一步控制 CPU 时钟，Clk_instr 只有在 Clk_CPU 为高电平且 sw_i[1] 为低电平时有效
assign Clk_instr = Clk_CPU & ~sw_i[1];

// 数码管显示数据寄存器
reg [63:0] display_data;
// LED 
reg [5:0] led_data_addr;
reg [63:0] led_disp_data;
// ROM
wire [31:0] instr;
wire [31:0] rom_addr;
// 通用寄存器数据寄存器
reg [31:0] reg_data;
reg [31:0] reg_addr;
// ALU数据寄存器
reg [31:0] alu_disp_data;
reg [2:0] alu_addr;
// 数据存储器数据寄存器
reg [31:0] dmem_data;
reg [7:0] dmem_addr;

// 定义LED 指令存储器 数据存储器 三者数据存储的数量
parameter LED_DATA_NUM = 19; // 共存储 19 组 LED 数据
parameter IM_CODE_NUM = 12; // 指令存储器中有 12 条指令
parameter DM_DATA_NUM = 16; // 数据存储器中有 16 组数据

// LED 数据存储器：用于存储 19 组 64 位数据，LED_DATA[0] 到 LED_DATA[18]
reg [63:0] LED_DATA [18:0];

// 定义所有连线
wire [6:0] Op;         // 操作码
wire [6:0] Funct7;     // 功能码（高 7 位）
wire [2:0] Funct3;     // 功能码（中间 3 位）
wire [11:0] iimm;      // I 型指令立即数
wire [11:0] simm;      // S 型指令立即数
wire [11:0] bimm;      // B 型指令立即数
wire [19:0] jimm;      // J 型指令立即数
wire [5:0] EXTOp;      // 扩展操作码
wire [31:0] immout;    // 立即数输出
wire [4:0] rs1, rs2;   // 源寄存器地址
wire [4:0] rd;         // 目标寄存器地址
reg [31:0] WD;         // 写入数据寄存器
wire [1:0] WDSel;      // 写入数据选择
wire RegWrite;         // 寄存器写使能
wire [31:0] RD1, RD2;  // 寄存器读数据
wire [31:0] A, B;      // ALU 操作数
wire ALUSrc;           // ALU 源选择
wire [4:0] ALUOp;      // ALU 操作码
wire [31:0] aluout;    // ALU 输出
wire Zero;             // ALU 零标志
wire MemWrite;         // 数据存储器写使能
wire [6:0] dm_addr;    // 数据存储器地址
wire [31:0] dm_din;    // 数据存储器输入
wire [2:0] DMType;     // 数据存储器访问类型
wire [31:0] dm_dout;   // 数据存储器输出
wire [2:0] NPCOp;      // 下一个 PC 的操作码
wire [31:0] PCout;     // 当前 PC 值
wire [31:0] NPC;       // 计算的下一个 PC

// 初始化信号连接
assign Op = instr[6:0];                        // 提取操作码
assign Funct7 = instr[31:25];                  // 提取功能码（高 7 位）
assign Funct3 = instr[14:12];                  // 提取功能码（中间 3 位）
assign rs1 = instr[19:15];                     // 提取源寄存器 1 地址
assign rs2 = instr[24:20];                     // 提取源寄存器 2 地址
assign rd = instr[11:7];                       // 提取目标寄存器地址
assign iimm = instr[31:20];                    // 提取 I 型指令立即数
assign simm = {instr[31:25], instr[11:7]};     // 拼接 S 型指令立即数
assign bimm = {instr[31], instr[7], instr[30:25], instr[11:8]}; // 拼接 B 型指令立即数
assign jimm = {instr[31], instr[19:12], instr[20], instr[30:21]}; // 拼接 J 型指令立即数

// 控制 LED 数据的显示逻辑
always @(posedge Clk_CPU or negedge rstn) begin
    if (!rstn) begin
        led_data_addr <= 6'd0;        // 复位时将 LED 数据地址设为 0
        led_disp_data <= 64'b1;       // 初始显示数据设为 1
    end else if (sw_i[0] == 1'b1) begin
        if (led_data_addr == LED_DATA_NUM) begin
            led_data_addr <= 6'd0;    // 如果达到最大地址，循环回到 0
            led_disp_data <= 64'b1;   // 重置显示数据
        end else begin
            led_disp_data <= LED_DATA[led_data_addr]; // 读取 LED 数据
            led_data_addr <= led_data_addr + 1'b1;    // 地址递增
        end
    end else begin
        led_data_addr <= led_data_addr; // 保持当前地址不变
    end
end

// 控制寄存器地址的递增和数据读取
always @(posedge Clk_CPU or negedge rstn) begin
    if (!rstn)
        reg_addr <= 0;                 // 复位时将寄存器地址设为 0
    else if (sw_i[13] == 1'b1) begin
        if (reg_addr == 31)
            reg_addr <= 0;             // 达到最大地址后循环回到 0
        else
            reg_addr <= reg_addr + 1;  // 地址递增
        reg_data = U_RF.rf[reg_addr];  // 读取对应地址的寄存器数据
    end
end

// 控制 ALU 数据的显示逻辑
always @(posedge Clk_CPU) begin
    alu_addr = alu_addr + 1'b1;        // ALU 地址递增
    case (alu_addr)
        3'b001: alu_disp_data = U_alu.A;       // 显示 ALU 输入 A
        3'b010: alu_disp_data = U_alu.B;       // 显示 ALU 输入 B
        3'b011: alu_disp_data = U_alu.C;       // 显示 ALU 计算结果 C
        3'b100: alu_disp_data = U_alu.Zero;    // 显示 ALU 零标志
        default: alu_disp_data = 32'hffffffff; // 默认显示全 1
    endcase
end

// 控制数据存储器地址和数据的显示逻辑
always @(posedge Clk_CPU or negedge rstn) begin
    if (!rstn) begin
        dmem_addr <= 0;               // 复位时地址清零
        dmem_data <= 32'hFFFFFFFF;    // 初始化数据为全 1
    end else if (sw_i[11] == 1'b1) begin
        if (dmem_addr == 16)          // 如果达到最大地址，循环回到 0
            dmem_addr <= 4'b0;
        else begin
            dmem_data <= U_DM.dmem[dmem_addr]; // 从数据存储器读取数据
            dmem_addr <= dmem_addr + 1'b1;    // 地址递增
        end
    end
end

// 初始化 LED 数据存储器
initial begin
    LED_DATA[0] = 64'hC6F6F6F0C6F6F6F0; 
    LED_DATA[1] = 64'hF9F6F6CFF9F6F6CF; 
    LED_DATA[2] = 64'hFFC6F0FFFFC6F0FF; 
    LED_DATA[3] = 64'hFFC0FFFFFFC0FFFF; 
    LED_DATA[4] = 64'hFFA3FFFFFFA3FFFF; 
    LED_DATA[5] = 64'hFFFFA3FFFFFFA3FF; 
    LED_DATA[6] = 64'hFFFF9CFFFFFF9CFF; 
    LED_DATA[7] = 64'hFF9EBCFFFF9EBCFF; 
    LED_DATA[8] = 64'hFF9CFFFFFF9CFFFF; 
    LED_DATA[9] = 64'hFFC0FFFFFFC0FFFF; 
    LED_DATA[10] = 64'hFFA3FFFFFFA3FFFF; 
    LED_DATA[11] = 64'hFFA7B3FFFFA7B3FF; 
    LED_DATA[12] = 64'hFFC6F0FFFFC6F0FF; 
    LED_DATA[13] = 64'hF9F6F6CFF9F6F6CF; 
    LED_DATA[14] = 64'h9EBEBEBC9EBEBEBC; 
    LED_DATA[15] = 64'h2737373327373733; 
    LED_DATA[16] = 64'h505454EC505454EC; 
    LED_DATA[17] = 64'h744454F8744454F8; 
    LED_DATA[18] = 64'h0062080000620800; 
end

// 根据开关选择显示的数据源
always @(sw_i) begin
    if (sw_i[0] == 1'b0) begin  // 如果 sw_i[0] 为 0，选择显示不同的数据源
        case (sw_i[14:11])      // 根据 sw_i[14:11] 选择显示内容
            4'b1000: display_data = instr;     // 显示指令存储器中的数据（ROM）
            4'b0100: display_data = reg_data;  // 显示寄存器文件中的数据（RF）
            4'b0010: display_data = alu_disp_data; // 显示 ALU 的数据
            4'b0001: display_data = dmem_data; // 显示数据存储器中的数据（DMEM）
            default: display_data = instr;  // 默认显示指令数据
        endcase
    end else begin
        display_data = led_disp_data;  // 如果 sw_i[0] 为 1，显示 LED 数据
    end
end
    
// 下面是各个模块的实例化    
// 控制模块实例化
ctrl U_ctrl(
    .Op(Op),              // 操作码
    .Funct7(Funct7),      // 功能码（高 7 位）
    .Funct3(Funct3),      // 功能码（中间 3 位）
    .Zero(Zero),          // ALU 零标志
    .RegWrite(RegWrite),  // 寄存器写使能信号
    .MemWrite(MemWrite),  // 数据存储器写使能信号
    .EXTOp(EXTOp),        // 扩展操作码
    .ALUOp(ALUOp),        // ALU 操作码
    .ALUSrc(ALUSrc),      // ALU 源选择
    .DMType(DMType),      // 数据存储器类型
    .WDSel(WDSel),        // 写数据选择
    .NPCOp(NPCOp)         // 下一个 PC 操作码
);

// 扩展模块实例化
EXT U_EXT(
    .iimm(iimm),          // I 型立即数
    .simm(simm),          // S 型立即数
    .bimm(bimm),          // B 型立即数
    .jimm(jimm),          // J 型立即数
    .EXTOp(EXTOp),        // 扩展操作码
    .immout(immout)       // 输出的扩展立即数
);

// PC & NPC 模块实例化
PC U_PC(
    .clk(Clk_CPU),        // 时钟信号
    .rstn(rstn),          // 复位信号
    .sw_i(sw_i),          // 控制开关输入
    .PC(rom_addr),        // 读取的 ROM 地址
    .NPC(NPC),            // 计算的下一个 PC 值
    .PCout(rom_addr)      // 当前 PC 输出
);

NPC U_NPC(
    .PC(rom_addr),        // 当前 PC 值
    .NPCOp(NPCOp),        // NPC 操作码
    .immout(immout),      // 扩展后的立即数
    .aluout(aluout),      // ALU 输出
    .NPC(NPC)             // 计算的下一个 PC 值
);

// 7 段数码管显示模块实例化
seg7x16 u_seg7x16(
    .clk(clk),            // 时钟信号
    .rstn(rstn),          // 复位信号
    .i_data(display_data), // 输入显示数据
    .disp_mode(sw_i[0]),  // 显示模式（由 sw_i[0] 控制）
    .o_seg(disp_seg_o),   // 数码管段选输出
    .o_sel(disp_an_o)     // 数码管位选输出
);

// ROM 实例化
dist_mem_gen_0 U_IM (
    .a(rom_addr[7:2]),    // ROM 地址（低 6 位用于地址选择）
    .spo(instr)           // 读取的指令数据
);

// 寄存器文件（RF）模块实例化
always@(*) begin
    case(WDSel)
    2'b00: WD <= aluout;  // 从 ALU 输出写数据
    2'b01: WD <= dm_dout; // 从数据存储器输出写数据
    2'b10: WD <= rom_addr + 4; // 从 PC+4（下一指令地址）写数据
    endcase
end

RF U_RF(
    .clk(Clk_CPU),        // 时钟信号
    .rstn(rstn),          // 复位信号
    .RFwr(RegWrite),      // 寄存器写使能
    .A1(rs1),             // 源寄存器 1 地址
    .A2(rs2),             // 源寄存器 2 地址
    .A3(rd),              // 目标寄存器地址
    .WD(WD),              // 写入数据
    .RD1(RD1),            // 读取数据 1
    .RD2(RD2)             // 读取数据 2
);

// ALU 模块实例化
assign B = (ALUSrc) ? immout : RD2; // ALU 输入选择：立即数或寄存器 2 数据
alu U_alu(
    .A(RD1),             // ALU 输入 A（来自寄存器 1）
    .B(B),               // ALU 输入 B（来自寄存器 2 或立即数）
    .ALUOp(ALUOp),       // ALU 操作码
    .C(aluout),          // ALU 输出
    .Zero(Zero)          // ALU 零标志
);

// 数据存储器（DM）模块实例化
assign dm_addr = aluout[6:0];  // 数据存储器地址（取 ALU 输出的低 7 位）

dm U_DM (
    .clk(Clk_instr),     // 时钟信号
    .rstn(rstn),         // 复位信号
    .DMWr(MemWrite),     // 数据存储器写使能
    .addr(dm_addr),      // 数据存储器地址
    .din(RD2),           // 写入数据（来自寄存器 2）
    .DMType(DMType),     // 数据存储器访问类型
    .dout(dm_dout)       // 数据存储器输出
);
    
endmodule

// 寄存器文件 (RF) 模块
module RF(
    input clk,                  // 时钟信号
    input rstn,                 // 复位信号，低有效
    input RFwr,                 // 寄存器写使能信号
    input [4:0] A1, A2, A3,     // 读取的寄存器地址，A1 和 A2 为读地址，A3 为写地址
    input [31:0] WD,            // 写数据
    output [31:0] RD1, RD2      // 读出的寄存器数据
);
    reg [31:0] rf[31:0];         // 32 个 32 位的寄存器文件
    integer i;

    // 时钟边缘触发的寄存器写操作
    always@(posedge clk or negedge rstn) begin
        if (!rstn) begin
            // 复位时，寄存器文件的所有寄存器值为其索引值
            for (i = 0; i < 32; i = i + 1) begin
                rf[i] = i;
            end
        end else begin
            if (RFwr && A3) begin
                // 写寄存器，如果 RFwr 为 1 且 A3 地址有效
                rf[A3] <= WD;
            end
        end
    end

    // 读取寄存器，A1 和 A2 为读取地址，若地址为 0，则返回 0
    assign RD1 = (A1 != 0) ? rf[A1] : 0;
    assign RD2 = (A2 != 0) ? rf[A2] : 0;
    
endmodule

// ALU 操作码定义
`define ALUOp_nop 5'b00000
`define ALUOp_lui 5'b00001
`define ALUOp_auipc 5'b00010
`define ALUOp_add 5'b00011
`define ALUOp_sub 5'b00100

// ALU 模块，用于执行算术逻辑运算
module alu(
    input signed [31:0] A, B,  // ALU 输入操作数 A 和 B
    input [4:0] ALUOp,         // ALU 控制信号
    output reg signed [31:0] C, // ALU 结果
    output reg Zero            // Zero 标志，表示运算结果是否为 0
);

    // 根据 ALU 操作码进行运算
    always @(*) begin
        C = 0; // 初始化结果，避免潜在的锁存器

        case (ALUOp)
            `ALUOp_add: C = A + B;  // 加法运算
            `ALUOp_sub: C = A - B;  // 减法运算
            // 其他操作可以在此处添加
            default: C = 0;         // 默认情况下为 0
        endcase

        // 设置 Zero 标志，若结果为 0，则 Zero 为 1
        Zero = (C == 0) ? 1 : 0;
    end

endmodule

// 数据存储器 (DM) 模块
module dm (
    input clk,                   // 时钟信号
    input rstn,                  // 复位信号，低有效
    input DMWr,                  // 数据存储器写使能信号
    input [6:0] addr,            // 存储器地址
    input [31:0] din,            // 写入数据
    input [2:0] DMType,          // 数据存储类型（字节、半字、字等）
    output reg [31:0] dout       // 读取的数据
);

    // 定义不同的数据存储类型
    `define dm_word 3'b000
    `define dm_halfword 3'b001
    `define dm_halfword_unsigned 3'b010
    `define dm_byte 3'b011
    `define dm_byte_unsigned 3'b100

    reg [7:0] dmem[127:0];  // 数据存储器，128 字节存储空间
    integer i;

    // 时钟边缘触发的存储器写操作
    always@(posedge clk or negedge rstn) begin
        if (!rstn) begin
            // 复位时，清空所有存储器数据
            for (i = 0; i < 128; i = i + 1) begin
                dmem[i] = 0;
            end
        end else begin
            if (DMWr == 1'b1) begin
                // 存储器写操作，根据 DMType 选择写入数据的方式
                case (DMType)
                    `dm_word: begin
                        dmem[addr] <= din[7:0];
                        dmem[addr+1] <= din[15:8];
                        dmem[addr+2] <= din[23:16];
                        dmem[addr+3] <= din[31:24];
                    end
                    `dm_halfword: begin
                        dmem[addr] <= din[7:0];
                        dmem[addr+1] <= din[15:8];
                    end
                    `dm_halfword_unsigned: begin
                        dmem[addr] <= din[7:0];
                        dmem[addr+1] <= din[15:8];
                    end
                    `dm_byte: dmem[addr] <= din[7:0];
                    `dm_byte_unsigned: dmem[addr] <= din[7:0];
                endcase
            end
        end
    end

    // 根据 DMType 读取数据
    always @(*) begin
        case (DMType)
            // 读取一个字
            `dm_word: dout <= {dmem[addr+3], dmem[addr+2], dmem[addr+1], dmem[addr]}; 
            // 读取一个有符号半字
            `dm_halfword: dout <= {{16{dmem[addr+1][7]}}, dmem[addr+1], dmem[addr]}; 
            // 读取一个无符号半字
            `dm_halfword_unsigned: dout <= {16'h0000, dmem[addr+1], dmem[addr]}; 
            // 读取一个字节（有符号扩展）
            `dm_byte: dout <= {{24{dmem[addr][7]}}, dmem[addr]}; 
            // 读取一个字节（无符号扩展）
            `dm_byte_unsigned: dout <= {24'h000000, dmem[addr]}; 
        endcase
    end
endmodule

// CTRL模块
module ctrl(
    input [6:0] Op,               // 指令的操作码 (Opcode)
    input [6:0] Funct7,            // 指令的 Funct7 字段（用于 R 类型指令）
    input [2:0] Funct3,            // 指令的 Funct3 字段（用于 R/I/S/B 类型指令）
    input Zero,                    // ALU 的 Zero 标志（用于判断分支是否跳转）
    output RegWrite,               // 寄存器写使能信号
    output MemWrite,               // 数据存储器写使能信号
    output [5:0] EXTOp,            // 扩展操作类型
    output [4:0] ALUOp,            // ALU 操作类型
    output ALUSrc,                 // ALU 第二操作数选择信号（选择立即数或寄存器数据）
    output [2:0] DMType,           // 数据存储器访问类型（字节、半字、字等）
    output [1:0] WDSel,            // 写回数据选择信号
    output [2:0] NPCOp             // 下一个程序计数器 (NPC) 操作类型
);

// R 类型指令的条件
wire rtype = ~Op[6] & Op[5] & Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];

// ADD 指令条件 (R 类型指令)
wire i_add = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] & 
            ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

// SUB 指令条件 (R 类型指令)
wire i_sub = rtype & ~Funct7[6] & Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] & 
            ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

// I 类型加载指令的条件
wire itype_l = ~Op[6] & ~Op[5] & ~Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];

// I 类型加载指令中的 LB、LH、LW 指令
wire i_lb = itype_l & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
wire i_lh = itype_l & ~Funct3[2] & ~Funct3[1] & Funct3[0];
wire i_lw = itype_l & ~Funct3[2] & Funct3[1] & ~Funct3[0];

// I 类型寄存器操作指令的条件
wire itype_r = ~Op[6] & ~Op[5] & Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];

// I 类型 ADDI 指令
wire i_addi = itype_r & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

// S 类型存储指令的条件
wire stype = ~Op[6] & Op[5] & ~Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];

// S 类型中的 SB、SH、SW 指令
wire i_sb = stype & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
wire i_sh = stype & ~Funct3[2] & ~Funct3[1] & Funct3[0];
wire i_sw = stype & ~Funct3[2] & Funct3[1] & ~Funct3[0];

// UJ 类型跳转指令的条件
wire i_jal = Op[6] & Op[5] & ~Op[4] & Op[3] & Op[2] & Op[1] & Op[0];
wire i_jalr = Op[6] & Op[5] & ~Op[4] & ~Op[3] & Op[2] & Op[1] & Op[0];

// SB 类型条件分支指令的条件
wire sbtype = Op[6] & Op[5] & ~Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];
wire i_beq = sbtype & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

// 寄存器写使能信号：R 类型、I 类型加载指令、I 类型寄存器指令、跳转指令（JAL、JALR）都会进行寄存器写操作
assign RegWrite = rtype | itype_l | itype_r | i_jal | i_jalr;

// 数据存储器写使能信号：仅 S 类型指令需要写入数据存储器
assign MemWrite = stype;

// 扩展操作类型信号（EXTOp）
// EXTOp 控制不同的扩展方式（立即数扩展类型等）
assign EXTOp[5] = 0;
assign EXTOp[4] = itype_l | i_addi | i_jalr;
assign EXTOp[3] = stype;
assign EXTOp[2] = sbtype;
assign EXTOp[1] = 0;
assign EXTOp[0] = i_jal;

// ALU 操作信号（ALUOp）
// ALUOp 用于控制 ALU 执行哪种运算
assign ALUOp[4] = 0;
assign ALUOp[3] = 0;
assign ALUOp[2] = i_beq;  // 分支指令使用比较运算（如 BEQ）
assign ALUOp[1] = i_add | itype_l | i_addi | stype | i_jalr;
assign ALUOp[0] = i_add | itype_l | i_addi | stype | i_jalr;

// ALU 第二操作数来源选择信号（ALUSrc）
// ALUSrc 控制 ALU 是否使用立即数作为第二操作数
assign ALUSrc = itype_l | itype_r | stype | i_jal | i_jalr;

// 数据存储器访问类型信号（DMType）
// DMType 控制数据存储器的访问方式（字节、半字或字等）
assign DMType[1] = i_lb | i_sb;
assign DMType[0] = i_lb | i_lh | i_sb | i_sh;

// 写回数据选择信号（WDSel）
// WDSel 控制写回寄存器的数据来源，来自于 JAL、JALR 或加载指令
assign WDSel[1] = i_jal | i_jalr;
assign WDSel[0] = itype_l;  // 仅加载指令使用数据存储器的数据

// 下一个程序计数器操作类型信号（NPCOp）
// NPCOp 控制程序计数器的更新方式
assign NPCOp[2] = i_jalr;   // JALR 使用寄存器值作为目标地址
assign NPCOp[1] = i_jal;    // JAL 使用当前 PC + 4 作为目标地址
assign NPCOp[0] = sbtype & Zero;  // 分支指令根据 Zero 标志来判断是否跳转

endmodule

// EXT 扩展模块
`define EXT_CTRL_ITYPE 6'b010000   // I 类型指令扩展控制信号
`define EXT_CTRL_STYPE 6'b001000   // S 类型指令扩展控制信号
`define EXT_CTRL_BTYPE 6'b000100   // B 类型指令扩展控制信号
`define EXT_CTRL_JTYPE 6'b000001   // J 类型指令扩展控制信号

module EXT(
    input [11:0] iimm,             // I 类型指令的立即数
    input [11:0] simm,             // S 类型指令的立即数
    input [11:0] bimm,             // B 类型指令的立即数
    input [19:0] jimm,             // J 类型指令的立即数
    input [5:0] EXTOp,             // 扩展操作类型（控制信号）
    output reg [31:0] immout       // 扩展后的立即数输出
);

always@(*) begin
    case(EXTOp) 
    `EXT_CTRL_ITYPE: immout <= {{20{iimm[11]}}, iimm};  // I 类型立即数扩展（符号扩展）
    `EXT_CTRL_STYPE: immout <= {{20{simm[11]}}, simm};  // S 类型立即数扩展（符号扩展）
    `EXT_CTRL_BTYPE: immout <= {{19{bimm[11]}}, bimm, 1'b0};  // B 类型立即数扩展（符号扩展，末尾补零）
    `EXT_CTRL_JTYPE: immout <= {{11{jimm[19]}}, jimm, 1'b0};  // J 类型立即数扩展（符号扩展，末尾补零）
    default: immout <= 32'h00000000;  // 默认情况下输出全 0
    endcase
end

endmodule

// PC 程序计数器模块
module PC(
    input clk,                     // 时钟信号
    input rstn,                    // 复位信号
    input [15:0] sw_i,             // 开关输入，用于控制程序计数器
    input [31:0] PC,               // 当前程序计数器的值
    input [31:0] NPC,              // 下一个程序计数器的值（NPC）
    output reg [31:0] PCout        // 输出的程序计数器值
);

always@(posedge clk or negedge rstn) begin
    if(!rstn) begin
        PCout <= 32'h00000000;  // 复位时，PC 输出清零
    end else begin
        if(sw_i[1] == 1'b0) begin
            PCout <= NPC;        // 如果开关控制为 0，使用 NPC 值
        end else begin
            PCout <= PC;         // 如果开关控制为 1，保持原 PC 值
        end
        // 如果 PC 达到指定值，重置为 0
        if(PCout == 32'h00000048) begin
            PCout <= 32'h00000000;
        end
    end
end

endmodule

// NPC 下一个程序计数器模块
`define NPC_PLUS4 3'b000   // PC + 4 操作
`define NPC_BRANCH 3'b001  // 分支跳转操作
`define NPC_JAL 3'b010     // JAL 指令跳转操作
`define NPC_JALR 3'b100    // JALR 指令跳转操作

module NPC(
    input [31:0] PC,            // 当前程序计数器的值
    input [2:0] NPCOp,          // 下一个程序计数器操作类型
    input [31:0] immout,        // 扩展后的立即数
    input [31:0] aluout,        // ALU 运算结果（用于 JALR）
    output reg [31:0] NPC       // 输出的下一个程序计数器值
);

always@(*) begin
    case(NPCOp)
    `NPC_PLUS4: NPC <= PC + 4;       // 普通情况下，PC + 4
    `NPC_BRANCH: NPC <= PC + immout; // 分支指令：PC + 扩展后的立即数
    `NPC_JAL: NPC <= PC + immout;    // JAL 指令：PC + 扩展后的立即数
    `NPC_JALR: NPC <= aluout;        // JALR 指令：跳转到 ALU 计算出的地址
    endcase
end

endmodule

// 提前给出的七段数码管模块
module seg7x16(
    input clk,
    input rstn,
    input disp_mode,
    input[63:0]i_data,
    output [7:0] o_seg,
    output [7:0] o_sel
    );
    
    reg [14:0]cnt;
    wire seg7_clk;
    
    always@(posedge clk,negedge rstn)
        if(!rstn)
            cnt<=0;
        else
            cnt <=cnt + 1'b1;
            assign seg7_clk = cnt[14];
reg [2:0] seg7_addr;

always@(posedge seg7_clk,negedge rstn)
        if(!rstn)
            seg7_addr<=0;
        else
             seg7_addr<=seg7_addr + 1'b1;
reg [7:0] o_sel_r;

always@(*)
    case(seg7_addr)
        7:o_sel_r=8'b01111111;
        6:o_sel_r=8'b10111111;
        5:o_sel_r=8'b11011111;
        4:o_sel_r=8'b11101111;
        3:o_sel_r=8'b11110111;
        2:o_sel_r=8'b11111011;
        1:o_sel_r=8'b11111101;
        0:o_sel_r=8'b11111110;
        endcase
    reg[63:0] i_data_store;
    always@(posedge clk,negedge rstn)
        if(!rstn)
            i_data_store<=0;
        else
             i_data_store<=i_data;
    reg[7:0]seg_data_r;
    always@(*)
        if(disp_mode==1'b0) begin
            case(seg7_addr)
                0:seg_data_r = i_data_store[3:0];
                1:seg_data_r = i_data_store[7:4];
                2:seg_data_r = i_data_store[11:8];
                3:seg_data_r = i_data_store[15:12];
                4:seg_data_r = i_data_store[19:16];
                5:seg_data_r = i_data_store[23:20];
                6:seg_data_r = i_data_store[27:24];
                7:seg_data_r = i_data_store[31:28];
            endcase end
         else begin
            case(seg7_addr)
                0:seg_data_r = i_data_store[7:0];
                1:seg_data_r = i_data_store[15:8];
                2:seg_data_r = i_data_store[23:16];
                3:seg_data_r = i_data_store[31:24];
                4:seg_data_r = i_data_store[39:32];
                5:seg_data_r = i_data_store[47:40];
                6:seg_data_r = i_data_store[55:48];
                7:seg_data_r = i_data_store[63:56];
            endcase end
    reg [7:0] o_seg_r;
    always@(posedge clk,negedge rstn)
        if(!rstn)
            o_seg_r <= 8'hff;
        else if(disp_mode==1'b0) begin
            case(seg_data_r)
            4'h0:o_seg_r <= 8'hC0;
            4'h1:o_seg_r <= 8'hF9;
            4'h2:o_seg_r <= 8'hA4;
            4'h3:o_seg_r <= 8'hB0;
            4'h4:o_seg_r <= 8'h99;
            4'h5:o_seg_r <= 8'h92;
            4'h6:o_seg_r <= 8'h82;
            4'h7:o_seg_r <= 8'hF8;
            4'h8:o_seg_r <= 8'h80;
            4'h9:o_seg_r <= 8'h90;
            4'hA:o_seg_r <= 8'h88;
            4'hB:o_seg_r <= 8'h83;
            4'hC:o_seg_r <= 8'hC6;
            4'hD:o_seg_r <= 8'hA1;
            4'hE:o_seg_r <= 8'h86;
            4'hF:o_seg_r <= 8'h8E;
            endcase end
            else begin   
                o_seg_r <= seg_data_r;
                end
     assign o_sel =o_sel_r;
     assign o_seg =o_seg_r;
endmodule