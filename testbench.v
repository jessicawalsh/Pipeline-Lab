`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/28/2019 07:14:16 PM
// Design Name: 
// Module Name: testbench
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
module testbench();

reg clk_tb;

CPU CPU_tb(clk_tb);

always 
begin 
    #5 clk_tb = ~clk_tb;
end

initial
begin 
    clk_tb=1'b0;
    CPU_tb.PC1.previous=32'd96;
end
endmodule
