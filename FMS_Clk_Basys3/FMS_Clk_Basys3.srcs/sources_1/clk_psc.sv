`timescale 100ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.10.2025 11:21:34
// Design Name: 
// Module Name: clk_psc
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


module clk_psc( input logic clk,
                output logic clk_scaled
    );
    
    logic [31:0]myreg;
    always @(posedge clk)
        myreg +=1;
        
    assign clk_scaled = myreg[28];
endmodule
