`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/28/2019 05:59:48 PM
// Design Name: 
// Module Name: lab5
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module PC
(
    input clk,
    input [31:0] next,
    output reg [31:0] previous
);
    
    always @ (posedge clk)
	begin
		previous <= next;  
	end
endmodule

module adder(
    input [31:0] previous,
    output reg [31:0] next
    );
    always@(*)
    begin
         next = previous + 4;
    end
endmodule

module instMem(
   input [31:0] a,
   output reg [31:0] do
   );
   
   reg [31:0] MEM [511:0];
   
   initial
   begin
       MEM[100] <= 32'h8c220000;
       MEM[104] <= 32'h8c230004;
       MEM[108] <= 32'h8c240008;       
       MEM[112] <= 32'h8c25000c;
       MEM[116] <= 32'h8c260010;
   end
   
   always@(a) 
   begin
       do <= MEM[a]; 
   end
endmodule

module IFID(
    input [31:0] do,
    input clk,
    output reg [31:0] out
    );    
    
    always @(posedge clk)
          begin  
             out <= do;
          end
endmodule

module Control(
        input [31:0] do,
        output reg  wreg,
        output reg  m2reg,
        output reg  wmem,
        output reg [3:0] aluc,
        output reg aluimm,
        output reg regrt
    );
    
    reg [5:0] opcode;  
    
    always@(*)
    begin
     opcode = do[31:26];
    case(opcode)
      6'b100011:
      begin
        wreg = 1'b0;
        m2reg = 1'b0;
        wmem = 1'b0;
        aluc = 4'b0010;
        aluimm = 1'b1;
        regrt =  1'b1;
      end
     6'b100000:
     begin 
        wreg = 1'b0;
        m2reg = 1'b0;
        wmem = 1'b0;
        aluc = 4'b0010;
        aluimm = 1'b0;
        regrt = 1'b0;
     end       
    endcase
    end
endmodule

module Mux (regrt, do, out2);
input regrt;
input [31:0] do;
reg [4:0] rd;
reg [4:0] rt;
output reg [4:0] out2;

always @ (*)
begin
    rd = do[15:11];
    rt = do[20:16];
    
    case(regrt)
        0:out2 = rd;
        1:out2 = rt;
    endcase
end
endmodule

module signExtend(
     input [31:0] do,
     output reg [31:0] SE_immediate
   );
   
   reg  [15:0] immediate;    
   
   always@(*)
   begin 
      immediate = do[15:0];
      SE_immediate = {{16{immediate[15]}},immediate[15:0]}; 
   end 
endmodule

module regFile(
    input [31:0] do,
    input clk,
    input [4:0] wn,
    input  [31:0] d,
    input we,
    output  reg [31:0] qa, qb
    );
    reg [4:0] rs; reg [4:0]rt;
    
    reg [31:0] regs [31:0];
    integer x;
    initial
      begin
         for(x=0;x<32;x= x+1) begin
         regs[x] = 32'd0;
         end          
 
      end
      
    always@(*) 
    begin        
        rs = do[25:21];
        rt = do[20:16];
        qa <= regs[rs]; 
        qb <= regs[rt];
    end
    
    always@(negedge clk)
    begin 
    if (we==1'b1)
        regs[wn] <= d;
     end 
endmodule

module IDEXE(
       input clk,
       input wreg,
       input m2reg,
       input wmem,
       input [3:0] aluc,
       input aluimm,
       input [4:0] out2,
       input [31:0] qa,
       input [31:0] qb,
       input [31:0] imm,
       output reg out_wreg,
       output reg out_m2reg,
       output reg out_wmem,
       output reg [3:0] out_aluc,
       output reg out_aluimm,
       output reg [4:0] outmux_out,
       output reg [31:0] qa_out,
       output reg [31:0] qb_out,
       output reg [31:0] imm_out
    ); 
    always@(posedge clk) 
    begin
       out_wreg = wreg;
       out_m2reg = m2reg;
       out_wmem = wmem;
       out_aluc = aluc;
       out_aluimm = aluimm;
       outmux_out = out2;
       qa_out = qa;
       qb_out = qb;
       imm_out = imm;
    end
    
endmodule

module MUX2
(
    input ealuimm,
    input [31:0] imm_out,
    input [31:0] exe_qb,
    output reg [31:0] MUX2_out 
);

always @ (*)
    begin
    case(ealuimm)
        0:MUX2_out = exe_qb;
        1:MUX2_out = imm_out;
    endcase
    end
endmodule 

module ALU
(
    input [3:0] ealuc,
    input [31:0] exe_qa,
    input [31:0] MUX2_out,
    output reg [31:0] r
);

always@(*)
    begin 
    case(ealuc)            
            4'b0010: r = exe_qa + MUX2_out;           
     endcase
     end
endmodule

module DataMemory
(
    input [31:0] r_mem,
    input [31:0] exe_qb,
    input  mwmem,
    output reg [31:0] do
);

reg [31:0] DATAMEMORY [511:0];

initial begin 
    DATAMEMORY[0] = 32'hA00000AA; 
    DATAMEMORY[4] = 32'h10000011;
    DATAMEMORY[8] = 32'h20000022; 
    DATAMEMORY[12] = 32'h30000033;
    DATAMEMORY[16] = 32'h40000044;
    DATAMEMORY[20]= 32'h50000055;
    DATAMEMORY[24] = 32'h60000066;
    DATAMEMORY[28] = 32'h70000077;
    DATAMEMORY[32] = 32'h80000088;
    DATAMEMORY[36] = 32'h90000099;
end 

 always@(*) 
  begin
   if(mwmem == 1'b0)
        do <= DATAMEMORY[r_mem];
 end
endmodule

module EXE_MEM(
    input clk,
    input ewreg,
    input em2reg,
    input ewmem,
    input [4:0] exe_out2,
    input [31:0] r,
    input [31:0] exe_qb,
    output reg mwreg,
    output reg mm2reg,
    output reg mwmem,
    output reg [31:0] r_mem,
    output reg [31:0] mem_qb,
    output reg [4:0] mem_out2
);

always@(posedge clk) 
begin 
    mwreg = ewreg;
    mm2reg = em2reg; 
    mwmem = ewmem;
    r_mem = r;
    mem_qb = exe_qb; 
    mem_out2 = exe_out2;
end

endmodule

module MEM_WB(
    input clk,
    input mwreg,
    input mm2reg, 
    input [4:0] mem_out2,
    input [31:0] r_mem,
    input [31:0] do,
    output reg wwreg,
    output reg wm2reg,
    output reg [4:0] wb_out2,
    output reg [31:0] r_wb,
    output reg [31:0] do_wb
);

always@(posedge clk)
begin 
    wwreg = mwreg;
    wm2reg = mm2reg;
    wb_out2 = mem_out2;
    r_wb = r_mem;
    do_wb = do; 
end
endmodule 

module MUX3
(
    input wm2reg,
    input [31:0] r_wb,
    input [31:0] do_wb,
    output reg [31:0] MUX3_out
);

always @ (*)
    begin
    case(wm2reg)
        0:MUX3_out = r_wb;
        1:MUX3_out = do_wb;
    endcase
    end
endmodule    


module CPU( input main_clk);
    wire [31:0] pc_wire1;
    wire [31:0] pc_wire0;
    wire [31:0] do;
    wire [31:0] do_out;
    wire  wreg_wire;
    wire m2reg_wire;
    wire wmem_wire;
    wire [3:0] aluc_wire;
    wire aluimm_wire;
    wire regrt_wire;
    wire [4:0] out2_wire;
    wire [31:0] wire_read1;
    wire [31:0] wire_read2;
    wire [31:0] SE_immediate1;
    
    wire wreg_out;
    wire m2reg_out;
    wire wmem_out;
    wire [3:0] aluc_out;
    wire aluimm_out; 
    wire [4:0] Mux_out;
    wire [31:0] qa_out_1;
    wire [31:0] qb_out_1;
    wire [31:0] out_immediate;  
    
    wire [31:0] mux2_out_wire;  
    wire [31:0] r_wire;
    wire [31:0] r_out_wire;    
    wire mwreg_wire;
    wire mm2reg_wire;
    wire mwmem_wire;
    wire [31:0] qb_out_2;
    wire [4:0] mux_out_2;
    
    wire[31:0] DM_do;
    
    wire wwreg_wire;
    wire wm2reg_wire;
    wire[4:0] mux_out_3;
    wire[31:0] r_wb_wire;
    wire[31:0] do_wb_wire;
    
    wire wreg_output;
    
    wire [4:0] mux3_out_wire;
      
    PC PC1 (main_clk, pc_wire0, pc_wire1);
    adder A1 (pc_wire1, pc_wire0);
    instMem M1(pc_wire1, do);
    IFID F1 (do, main_clk, do_out);
    Control C1 (do_out, wreg_wire, m2reg_wire, wmem_wire, aluc_wire, aluimm_wire, regrt_wire);
    Mux Mux1 (regrt_wire, do_out, out2_wire);
    regFile RF1 (do_out,main_clk, mux_out_3, mux3_out_wire, wwreg_wire, wire_read1, wire_read2);
    signExtend SE1(do_out, SE_immediate1);
    IDEXE ID(main_clk, wreg_wire, m2reg_wire, wmem_wire, aluc_wire, aluimm_wire, out2_wire, wire_read1, wire_read2, SE_immediate1, wreg_out,
     m2reg_out, wmem_out, aluc_out,aluimm_out, Mux_out, qa_out_1, qb_out_1, out_immediate);  
    MUX2 mux_2 (aluimm_out, out_immediate, qb_out_1, mux2_out_wire);        
    ALU ALU1 (aluc_out, qa_out_1, mux2_out_wire, r_wire);
    EXE_MEM EXE(main_clk, wreg_out, m2reg_out,wmem_out, Mux_out, r_wire, qb_out_1,mwreg_wire, mm2reg_wire, mwmem_wire, r_out_wire, qb_out_2, mux_out_2);
    DataMemory DM1 (r_out_wire, qb_out_2, mwmem_wire, DM_do);
    MEM_WB MEM(main_clk, mwreg_wire, mm2reg_wire, mux_out_2, r_out_wire, DM_do, wwreg_wire, wm2reg_wire, mux_out_3, r_wb_wire, do_wb_wire);   
    MUX3 mux_3 (wm2reg_wire, r_wb_wire, do_wb_wire, mux3_out_wire);  
          
endmodule



