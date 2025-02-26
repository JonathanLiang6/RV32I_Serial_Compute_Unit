`timescale 1ns / 1ps 

module sccomp(
    input clk,                // ȫ��ʱ������
    input rstn,               // ȫ�ָ�λ���루�͵�ƽ��Ч��
    input [15:0] sw_i,        // ���������źţ����ڿ��ƹ���
    output [7:0] disp_seg_o,  // ����ܶ�ѡ���
    output [7:0] disp_an_o    // �����λѡ���
);

// ʱ�ӷ�Ƶ�Ĵ������������ɲ�ͬƵ�ʵ�ʱ���ź�
reg [31:0] clkdiv;          
wire Clk_CPU;              

// ʱ�ӷ�Ƶ�߼�����������ʱ�ӻ��½��ظ�λ�ź�ʱ���·�Ƶ�Ĵ���
always @(posedge clk or negedge rstn) begin
    if (!rstn)               // �����λ�ź���Ч���͵�ƽ��������Ƶ�Ĵ�������
        clkdiv <= 0;
    else                     // ���򣬷�Ƶ�Ĵ�������
        clkdiv <= clkdiv + 1'b1;
end

// CPU ʱ��ѡ���߼���
// ��� sw_i[15] Ϊ�ߵ�ƽ����ѡ������� clkdiv[27] ��Ϊ CPU ʱ�ӣ�����ѡ��Ͽ�� clkdiv[25]
assign Clk_CPU = (sw_i[15]) ? clkdiv[27] : clkdiv[25];
// ��� sw_i[1] �źŽ�һ������ CPU ʱ�ӣ�Clk_instr ֻ���� Clk_CPU Ϊ�ߵ�ƽ�� sw_i[1] Ϊ�͵�ƽʱ��Ч
assign Clk_instr = Clk_CPU & ~sw_i[1];

// �������ʾ���ݼĴ���
reg [63:0] display_data;
// LED 
reg [5:0] led_data_addr;
reg [63:0] led_disp_data;
// ROM
wire [31:0] instr;
wire [31:0] rom_addr;
// ͨ�üĴ������ݼĴ���
reg [31:0] reg_data;
reg [31:0] reg_addr;
// ALU���ݼĴ���
reg [31:0] alu_disp_data;
reg [2:0] alu_addr;
// ���ݴ洢�����ݼĴ���
reg [31:0] dmem_data;
reg [7:0] dmem_addr;

// ����LED ָ��洢�� ���ݴ洢�� �������ݴ洢������
parameter LED_DATA_NUM = 19; // ���洢 19 �� LED ����
parameter IM_CODE_NUM = 12; // ָ��洢������ 12 ��ָ��
parameter DM_DATA_NUM = 16; // ���ݴ洢������ 16 ������

// LED ���ݴ洢�������ڴ洢 19 �� 64 λ���ݣ�LED_DATA[0] �� LED_DATA[18]
reg [63:0] LED_DATA [18:0];

// ������������
wire [6:0] Op;         // ������
wire [6:0] Funct7;     // �����루�� 7 λ��
wire [2:0] Funct3;     // �����루�м� 3 λ��
wire [11:0] iimm;      // I ��ָ��������
wire [11:0] simm;      // S ��ָ��������
wire [11:0] bimm;      // B ��ָ��������
wire [19:0] jimm;      // J ��ָ��������
wire [5:0] EXTOp;      // ��չ������
wire [31:0] immout;    // ���������
wire [4:0] rs1, rs2;   // Դ�Ĵ�����ַ
wire [4:0] rd;         // Ŀ��Ĵ�����ַ
reg [31:0] WD;         // д�����ݼĴ���
wire [1:0] WDSel;      // д������ѡ��
wire RegWrite;         // �Ĵ���дʹ��
wire [31:0] RD1, RD2;  // �Ĵ���������
wire [31:0] A, B;      // ALU ������
wire ALUSrc;           // ALU Դѡ��
wire [4:0] ALUOp;      // ALU ������
wire [31:0] aluout;    // ALU ���
wire Zero;             // ALU ���־
wire MemWrite;         // ���ݴ洢��дʹ��
wire [6:0] dm_addr;    // ���ݴ洢����ַ
wire [31:0] dm_din;    // ���ݴ洢������
wire [2:0] DMType;     // ���ݴ洢����������
wire [31:0] dm_dout;   // ���ݴ洢�����
wire [2:0] NPCOp;      // ��һ�� PC �Ĳ�����
wire [31:0] PCout;     // ��ǰ PC ֵ
wire [31:0] NPC;       // �������һ�� PC

// ��ʼ���ź�����
assign Op = instr[6:0];                        // ��ȡ������
assign Funct7 = instr[31:25];                  // ��ȡ�����루�� 7 λ��
assign Funct3 = instr[14:12];                  // ��ȡ�����루�м� 3 λ��
assign rs1 = instr[19:15];                     // ��ȡԴ�Ĵ��� 1 ��ַ
assign rs2 = instr[24:20];                     // ��ȡԴ�Ĵ��� 2 ��ַ
assign rd = instr[11:7];                       // ��ȡĿ��Ĵ�����ַ
assign iimm = instr[31:20];                    // ��ȡ I ��ָ��������
assign simm = {instr[31:25], instr[11:7]};     // ƴ�� S ��ָ��������
assign bimm = {instr[31], instr[7], instr[30:25], instr[11:8]}; // ƴ�� B ��ָ��������
assign jimm = {instr[31], instr[19:12], instr[20], instr[30:21]}; // ƴ�� J ��ָ��������

// ���� LED ���ݵ���ʾ�߼�
always @(posedge Clk_CPU or negedge rstn) begin
    if (!rstn) begin
        led_data_addr <= 6'd0;        // ��λʱ�� LED ���ݵ�ַ��Ϊ 0
        led_disp_data <= 64'b1;       // ��ʼ��ʾ������Ϊ 1
    end else if (sw_i[0] == 1'b1) begin
        if (led_data_addr == LED_DATA_NUM) begin
            led_data_addr <= 6'd0;    // ����ﵽ����ַ��ѭ���ص� 0
            led_disp_data <= 64'b1;   // ������ʾ����
        end else begin
            led_disp_data <= LED_DATA[led_data_addr]; // ��ȡ LED ����
            led_data_addr <= led_data_addr + 1'b1;    // ��ַ����
        end
    end else begin
        led_data_addr <= led_data_addr; // ���ֵ�ǰ��ַ����
    end
end

// ���ƼĴ�����ַ�ĵ��������ݶ�ȡ
always @(posedge Clk_CPU or negedge rstn) begin
    if (!rstn)
        reg_addr <= 0;                 // ��λʱ���Ĵ�����ַ��Ϊ 0
    else if (sw_i[13] == 1'b1) begin
        if (reg_addr == 31)
            reg_addr <= 0;             // �ﵽ����ַ��ѭ���ص� 0
        else
            reg_addr <= reg_addr + 1;  // ��ַ����
        reg_data = U_RF.rf[reg_addr];  // ��ȡ��Ӧ��ַ�ļĴ�������
    end
end

// ���� ALU ���ݵ���ʾ�߼�
always @(posedge Clk_CPU) begin
    alu_addr = alu_addr + 1'b1;        // ALU ��ַ����
    case (alu_addr)
        3'b001: alu_disp_data = U_alu.A;       // ��ʾ ALU ���� A
        3'b010: alu_disp_data = U_alu.B;       // ��ʾ ALU ���� B
        3'b011: alu_disp_data = U_alu.C;       // ��ʾ ALU ������ C
        3'b100: alu_disp_data = U_alu.Zero;    // ��ʾ ALU ���־
        default: alu_disp_data = 32'hffffffff; // Ĭ����ʾȫ 1
    endcase
end

// �������ݴ洢����ַ�����ݵ���ʾ�߼�
always @(posedge Clk_CPU or negedge rstn) begin
    if (!rstn) begin
        dmem_addr <= 0;               // ��λʱ��ַ����
        dmem_data <= 32'hFFFFFFFF;    // ��ʼ������Ϊȫ 1
    end else if (sw_i[11] == 1'b1) begin
        if (dmem_addr == 16)          // ����ﵽ����ַ��ѭ���ص� 0
            dmem_addr <= 4'b0;
        else begin
            dmem_data <= U_DM.dmem[dmem_addr]; // �����ݴ洢����ȡ����
            dmem_addr <= dmem_addr + 1'b1;    // ��ַ����
        end
    end
end

// ��ʼ�� LED ���ݴ洢��
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

// ���ݿ���ѡ����ʾ������Դ
always @(sw_i) begin
    if (sw_i[0] == 1'b0) begin  // ��� sw_i[0] Ϊ 0��ѡ����ʾ��ͬ������Դ
        case (sw_i[14:11])      // ���� sw_i[14:11] ѡ����ʾ����
            4'b1000: display_data = instr;     // ��ʾָ��洢���е����ݣ�ROM��
            4'b0100: display_data = reg_data;  // ��ʾ�Ĵ����ļ��е����ݣ�RF��
            4'b0010: display_data = alu_disp_data; // ��ʾ ALU ������
            4'b0001: display_data = dmem_data; // ��ʾ���ݴ洢���е����ݣ�DMEM��
            default: display_data = instr;  // Ĭ����ʾָ������
        endcase
    end else begin
        display_data = led_disp_data;  // ��� sw_i[0] Ϊ 1����ʾ LED ����
    end
end
    
// �����Ǹ���ģ���ʵ����    
// ����ģ��ʵ����
ctrl U_ctrl(
    .Op(Op),              // ������
    .Funct7(Funct7),      // �����루�� 7 λ��
    .Funct3(Funct3),      // �����루�м� 3 λ��
    .Zero(Zero),          // ALU ���־
    .RegWrite(RegWrite),  // �Ĵ���дʹ���ź�
    .MemWrite(MemWrite),  // ���ݴ洢��дʹ���ź�
    .EXTOp(EXTOp),        // ��չ������
    .ALUOp(ALUOp),        // ALU ������
    .ALUSrc(ALUSrc),      // ALU Դѡ��
    .DMType(DMType),      // ���ݴ洢������
    .WDSel(WDSel),        // д����ѡ��
    .NPCOp(NPCOp)         // ��һ�� PC ������
);

// ��չģ��ʵ����
EXT U_EXT(
    .iimm(iimm),          // I ��������
    .simm(simm),          // S ��������
    .bimm(bimm),          // B ��������
    .jimm(jimm),          // J ��������
    .EXTOp(EXTOp),        // ��չ������
    .immout(immout)       // �������չ������
);

// PC & NPC ģ��ʵ����
PC U_PC(
    .clk(Clk_CPU),        // ʱ���ź�
    .rstn(rstn),          // ��λ�ź�
    .sw_i(sw_i),          // ���ƿ�������
    .PC(rom_addr),        // ��ȡ�� ROM ��ַ
    .NPC(NPC),            // �������һ�� PC ֵ
    .PCout(rom_addr)      // ��ǰ PC ���
);

NPC U_NPC(
    .PC(rom_addr),        // ��ǰ PC ֵ
    .NPCOp(NPCOp),        // NPC ������
    .immout(immout),      // ��չ���������
    .aluout(aluout),      // ALU ���
    .NPC(NPC)             // �������һ�� PC ֵ
);

// 7 ���������ʾģ��ʵ����
seg7x16 u_seg7x16(
    .clk(clk),            // ʱ���ź�
    .rstn(rstn),          // ��λ�ź�
    .i_data(display_data), // ������ʾ����
    .disp_mode(sw_i[0]),  // ��ʾģʽ���� sw_i[0] ���ƣ�
    .o_seg(disp_seg_o),   // ����ܶ�ѡ���
    .o_sel(disp_an_o)     // �����λѡ���
);

// ROM ʵ����
dist_mem_gen_0 U_IM (
    .a(rom_addr[7:2]),    // ROM ��ַ���� 6 λ���ڵ�ַѡ��
    .spo(instr)           // ��ȡ��ָ������
);

// �Ĵ����ļ���RF��ģ��ʵ����
always@(*) begin
    case(WDSel)
    2'b00: WD <= aluout;  // �� ALU ���д����
    2'b01: WD <= dm_dout; // �����ݴ洢�����д����
    2'b10: WD <= rom_addr + 4; // �� PC+4����һָ���ַ��д����
    endcase
end

RF U_RF(
    .clk(Clk_CPU),        // ʱ���ź�
    .rstn(rstn),          // ��λ�ź�
    .RFwr(RegWrite),      // �Ĵ���дʹ��
    .A1(rs1),             // Դ�Ĵ��� 1 ��ַ
    .A2(rs2),             // Դ�Ĵ��� 2 ��ַ
    .A3(rd),              // Ŀ��Ĵ�����ַ
    .WD(WD),              // д������
    .RD1(RD1),            // ��ȡ���� 1
    .RD2(RD2)             // ��ȡ���� 2
);

// ALU ģ��ʵ����
assign B = (ALUSrc) ? immout : RD2; // ALU ����ѡ����������Ĵ��� 2 ����
alu U_alu(
    .A(RD1),             // ALU ���� A�����ԼĴ��� 1��
    .B(B),               // ALU ���� B�����ԼĴ��� 2 ����������
    .ALUOp(ALUOp),       // ALU ������
    .C(aluout),          // ALU ���
    .Zero(Zero)          // ALU ���־
);

// ���ݴ洢����DM��ģ��ʵ����
assign dm_addr = aluout[6:0];  // ���ݴ洢����ַ��ȡ ALU ����ĵ� 7 λ��

dm U_DM (
    .clk(Clk_instr),     // ʱ���ź�
    .rstn(rstn),         // ��λ�ź�
    .DMWr(MemWrite),     // ���ݴ洢��дʹ��
    .addr(dm_addr),      // ���ݴ洢����ַ
    .din(RD2),           // д�����ݣ����ԼĴ��� 2��
    .DMType(DMType),     // ���ݴ洢����������
    .dout(dm_dout)       // ���ݴ洢�����
);
    
endmodule

// �Ĵ����ļ� (RF) ģ��
module RF(
    input clk,                  // ʱ���ź�
    input rstn,                 // ��λ�źţ�����Ч
    input RFwr,                 // �Ĵ���дʹ���ź�
    input [4:0] A1, A2, A3,     // ��ȡ�ļĴ�����ַ��A1 �� A2 Ϊ����ַ��A3 Ϊд��ַ
    input [31:0] WD,            // д����
    output [31:0] RD1, RD2      // �����ļĴ�������
);
    reg [31:0] rf[31:0];         // 32 �� 32 λ�ļĴ����ļ�
    integer i;

    // ʱ�ӱ�Ե�����ļĴ���д����
    always@(posedge clk or negedge rstn) begin
        if (!rstn) begin
            // ��λʱ���Ĵ����ļ������мĴ���ֵΪ������ֵ
            for (i = 0; i < 32; i = i + 1) begin
                rf[i] = i;
            end
        end else begin
            if (RFwr && A3) begin
                // д�Ĵ�������� RFwr Ϊ 1 �� A3 ��ַ��Ч
                rf[A3] <= WD;
            end
        end
    end

    // ��ȡ�Ĵ�����A1 �� A2 Ϊ��ȡ��ַ������ַΪ 0���򷵻� 0
    assign RD1 = (A1 != 0) ? rf[A1] : 0;
    assign RD2 = (A2 != 0) ? rf[A2] : 0;
    
endmodule

// ALU �����붨��
`define ALUOp_nop 5'b00000
`define ALUOp_lui 5'b00001
`define ALUOp_auipc 5'b00010
`define ALUOp_add 5'b00011
`define ALUOp_sub 5'b00100

// ALU ģ�飬����ִ�������߼�����
module alu(
    input signed [31:0] A, B,  // ALU ��������� A �� B
    input [4:0] ALUOp,         // ALU �����ź�
    output reg signed [31:0] C, // ALU ���
    output reg Zero            // Zero ��־����ʾ�������Ƿ�Ϊ 0
);

    // ���� ALU �������������
    always @(*) begin
        C = 0; // ��ʼ�����������Ǳ�ڵ�������

        case (ALUOp)
            `ALUOp_add: C = A + B;  // �ӷ�����
            `ALUOp_sub: C = A - B;  // ��������
            // �������������ڴ˴����
            default: C = 0;         // Ĭ�������Ϊ 0
        endcase

        // ���� Zero ��־�������Ϊ 0���� Zero Ϊ 1
        Zero = (C == 0) ? 1 : 0;
    end

endmodule

// ���ݴ洢�� (DM) ģ��
module dm (
    input clk,                   // ʱ���ź�
    input rstn,                  // ��λ�źţ�����Ч
    input DMWr,                  // ���ݴ洢��дʹ���ź�
    input [6:0] addr,            // �洢����ַ
    input [31:0] din,            // д������
    input [2:0] DMType,          // ���ݴ洢���ͣ��ֽڡ����֡��ֵȣ�
    output reg [31:0] dout       // ��ȡ������
);

    // ���岻ͬ�����ݴ洢����
    `define dm_word 3'b000
    `define dm_halfword 3'b001
    `define dm_halfword_unsigned 3'b010
    `define dm_byte 3'b011
    `define dm_byte_unsigned 3'b100

    reg [7:0] dmem[127:0];  // ���ݴ洢����128 �ֽڴ洢�ռ�
    integer i;

    // ʱ�ӱ�Ե�����Ĵ洢��д����
    always@(posedge clk or negedge rstn) begin
        if (!rstn) begin
            // ��λʱ��������д洢������
            for (i = 0; i < 128; i = i + 1) begin
                dmem[i] = 0;
            end
        end else begin
            if (DMWr == 1'b1) begin
                // �洢��д���������� DMType ѡ��д�����ݵķ�ʽ
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

    // ���� DMType ��ȡ����
    always @(*) begin
        case (DMType)
            // ��ȡһ����
            `dm_word: dout <= {dmem[addr+3], dmem[addr+2], dmem[addr+1], dmem[addr]}; 
            // ��ȡһ���з��Ű���
            `dm_halfword: dout <= {{16{dmem[addr+1][7]}}, dmem[addr+1], dmem[addr]}; 
            // ��ȡһ���޷��Ű���
            `dm_halfword_unsigned: dout <= {16'h0000, dmem[addr+1], dmem[addr]}; 
            // ��ȡһ���ֽڣ��з�����չ��
            `dm_byte: dout <= {{24{dmem[addr][7]}}, dmem[addr]}; 
            // ��ȡһ���ֽڣ��޷�����չ��
            `dm_byte_unsigned: dout <= {24'h000000, dmem[addr]}; 
        endcase
    end
endmodule

// CTRLģ��
module ctrl(
    input [6:0] Op,               // ָ��Ĳ����� (Opcode)
    input [6:0] Funct7,            // ָ��� Funct7 �ֶΣ����� R ����ָ�
    input [2:0] Funct3,            // ָ��� Funct3 �ֶΣ����� R/I/S/B ����ָ�
    input Zero,                    // ALU �� Zero ��־�������жϷ�֧�Ƿ���ת��
    output RegWrite,               // �Ĵ���дʹ���ź�
    output MemWrite,               // ���ݴ洢��дʹ���ź�
    output [5:0] EXTOp,            // ��չ��������
    output [4:0] ALUOp,            // ALU ��������
    output ALUSrc,                 // ALU �ڶ�������ѡ���źţ�ѡ����������Ĵ������ݣ�
    output [2:0] DMType,           // ���ݴ洢���������ͣ��ֽڡ����֡��ֵȣ�
    output [1:0] WDSel,            // д������ѡ���ź�
    output [2:0] NPCOp             // ��һ����������� (NPC) ��������
);

// R ����ָ�������
wire rtype = ~Op[6] & Op[5] & Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];

// ADD ָ������ (R ����ָ��)
wire i_add = rtype & ~Funct7[6] & ~Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] & 
            ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

// SUB ָ������ (R ����ָ��)
wire i_sub = rtype & ~Funct7[6] & Funct7[5] & ~Funct7[4] & ~Funct7[3] & ~Funct7[2] & ~Funct7[1] & ~Funct7[0] & 
            ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

// I ���ͼ���ָ�������
wire itype_l = ~Op[6] & ~Op[5] & ~Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];

// I ���ͼ���ָ���е� LB��LH��LW ָ��
wire i_lb = itype_l & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
wire i_lh = itype_l & ~Funct3[2] & ~Funct3[1] & Funct3[0];
wire i_lw = itype_l & ~Funct3[2] & Funct3[1] & ~Funct3[0];

// I ���ͼĴ�������ָ�������
wire itype_r = ~Op[6] & ~Op[5] & Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];

// I ���� ADDI ָ��
wire i_addi = itype_r & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

// S ���ʹ洢ָ�������
wire stype = ~Op[6] & Op[5] & ~Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];

// S �����е� SB��SH��SW ָ��
wire i_sb = stype & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];
wire i_sh = stype & ~Funct3[2] & ~Funct3[1] & Funct3[0];
wire i_sw = stype & ~Funct3[2] & Funct3[1] & ~Funct3[0];

// UJ ������תָ�������
wire i_jal = Op[6] & Op[5] & ~Op[4] & Op[3] & Op[2] & Op[1] & Op[0];
wire i_jalr = Op[6] & Op[5] & ~Op[4] & ~Op[3] & Op[2] & Op[1] & Op[0];

// SB ����������ָ֧�������
wire sbtype = Op[6] & Op[5] & ~Op[4] & ~Op[3] & ~Op[2] & Op[1] & Op[0];
wire i_beq = sbtype & ~Funct3[2] & ~Funct3[1] & ~Funct3[0];

// �Ĵ���дʹ���źţ�R ���͡�I ���ͼ���ָ�I ���ͼĴ���ָ���תָ�JAL��JALR��������мĴ���д����
assign RegWrite = rtype | itype_l | itype_r | i_jal | i_jalr;

// ���ݴ洢��дʹ���źţ��� S ����ָ����Ҫд�����ݴ洢��
assign MemWrite = stype;

// ��չ���������źţ�EXTOp��
// EXTOp ���Ʋ�ͬ����չ��ʽ����������չ���͵ȣ�
assign EXTOp[5] = 0;
assign EXTOp[4] = itype_l | i_addi | i_jalr;
assign EXTOp[3] = stype;
assign EXTOp[2] = sbtype;
assign EXTOp[1] = 0;
assign EXTOp[0] = i_jal;

// ALU �����źţ�ALUOp��
// ALUOp ���ڿ��� ALU ִ����������
assign ALUOp[4] = 0;
assign ALUOp[3] = 0;
assign ALUOp[2] = i_beq;  // ��ָ֧��ʹ�ñȽ����㣨�� BEQ��
assign ALUOp[1] = i_add | itype_l | i_addi | stype | i_jalr;
assign ALUOp[0] = i_add | itype_l | i_addi | stype | i_jalr;

// ALU �ڶ���������Դѡ���źţ�ALUSrc��
// ALUSrc ���� ALU �Ƿ�ʹ����������Ϊ�ڶ�������
assign ALUSrc = itype_l | itype_r | stype | i_jal | i_jalr;

// ���ݴ洢�����������źţ�DMType��
// DMType �������ݴ洢���ķ��ʷ�ʽ���ֽڡ����ֻ��ֵȣ�
assign DMType[1] = i_lb | i_sb;
assign DMType[0] = i_lb | i_lh | i_sb | i_sh;

// д������ѡ���źţ�WDSel��
// WDSel ����д�ؼĴ�����������Դ�������� JAL��JALR �����ָ��
assign WDSel[1] = i_jal | i_jalr;
assign WDSel[0] = itype_l;  // ������ָ��ʹ�����ݴ洢��������

// ��һ��������������������źţ�NPCOp��
// NPCOp ���Ƴ���������ĸ��·�ʽ
assign NPCOp[2] = i_jalr;   // JALR ʹ�üĴ���ֵ��ΪĿ���ַ
assign NPCOp[1] = i_jal;    // JAL ʹ�õ�ǰ PC + 4 ��ΪĿ���ַ
assign NPCOp[0] = sbtype & Zero;  // ��ָ֧����� Zero ��־���ж��Ƿ���ת

endmodule

// EXT ��չģ��
`define EXT_CTRL_ITYPE 6'b010000   // I ����ָ����չ�����ź�
`define EXT_CTRL_STYPE 6'b001000   // S ����ָ����չ�����ź�
`define EXT_CTRL_BTYPE 6'b000100   // B ����ָ����չ�����ź�
`define EXT_CTRL_JTYPE 6'b000001   // J ����ָ����չ�����ź�

module EXT(
    input [11:0] iimm,             // I ����ָ���������
    input [11:0] simm,             // S ����ָ���������
    input [11:0] bimm,             // B ����ָ���������
    input [19:0] jimm,             // J ����ָ���������
    input [5:0] EXTOp,             // ��չ�������ͣ������źţ�
    output reg [31:0] immout       // ��չ������������
);

always@(*) begin
    case(EXTOp) 
    `EXT_CTRL_ITYPE: immout <= {{20{iimm[11]}}, iimm};  // I ������������չ��������չ��
    `EXT_CTRL_STYPE: immout <= {{20{simm[11]}}, simm};  // S ������������չ��������չ��
    `EXT_CTRL_BTYPE: immout <= {{19{bimm[11]}}, bimm, 1'b0};  // B ������������չ��������չ��ĩβ���㣩
    `EXT_CTRL_JTYPE: immout <= {{11{jimm[19]}}, jimm, 1'b0};  // J ������������չ��������չ��ĩβ���㣩
    default: immout <= 32'h00000000;  // Ĭ����������ȫ 0
    endcase
end

endmodule

// PC ���������ģ��
module PC(
    input clk,                     // ʱ���ź�
    input rstn,                    // ��λ�ź�
    input [15:0] sw_i,             // �������룬���ڿ��Ƴ��������
    input [31:0] PC,               // ��ǰ�����������ֵ
    input [31:0] NPC,              // ��һ�������������ֵ��NPC��
    output reg [31:0] PCout        // ����ĳ��������ֵ
);

always@(posedge clk or negedge rstn) begin
    if(!rstn) begin
        PCout <= 32'h00000000;  // ��λʱ��PC �������
    end else begin
        if(sw_i[1] == 1'b0) begin
            PCout <= NPC;        // ������ؿ���Ϊ 0��ʹ�� NPC ֵ
        end else begin
            PCout <= PC;         // ������ؿ���Ϊ 1������ԭ PC ֵ
        end
        // ��� PC �ﵽָ��ֵ������Ϊ 0
        if(PCout == 32'h00000048) begin
            PCout <= 32'h00000000;
        end
    end
end

endmodule

// NPC ��һ�����������ģ��
`define NPC_PLUS4 3'b000   // PC + 4 ����
`define NPC_BRANCH 3'b001  // ��֧��ת����
`define NPC_JAL 3'b010     // JAL ָ����ת����
`define NPC_JALR 3'b100    // JALR ָ����ת����

module NPC(
    input [31:0] PC,            // ��ǰ�����������ֵ
    input [2:0] NPCOp,          // ��һ�������������������
    input [31:0] immout,        // ��չ���������
    input [31:0] aluout,        // ALU ������������ JALR��
    output reg [31:0] NPC       // �������һ�����������ֵ
);

always@(*) begin
    case(NPCOp)
    `NPC_PLUS4: NPC <= PC + 4;       // ��ͨ����£�PC + 4
    `NPC_BRANCH: NPC <= PC + immout; // ��ָ֧�PC + ��չ���������
    `NPC_JAL: NPC <= PC + immout;    // JAL ָ�PC + ��չ���������
    `NPC_JALR: NPC <= aluout;        // JALR ָ���ת�� ALU ������ĵ�ַ
    endcase
end

endmodule

// ��ǰ�������߶������ģ��
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