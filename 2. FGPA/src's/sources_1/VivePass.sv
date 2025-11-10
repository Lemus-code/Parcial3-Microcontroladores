`timescale 100ns / 1ps

module VivePass(
    input  logic clk,
    input  logic reset,
    input  logic [1:0] sw, 
    input  logic btnU, //change to clk
    output logic led,
    output logic [3:0] an,
    output logic [6:0] seg,
    output logic dp
);

//Plantilla de L
typedef enum logic [1:0] {Digits_0, Digit_1, Digits_2, Digits_3} outtype;
outtype l;

typedef enum logic {Abrir, Cerrar} outtypet;
outtypet A_t;

//Señales internas
logic [1:0] L_out;
logic c_out;
logic clk2;
logic select = 1'b0;
logic btnU_d;

always_ff @(posedge clk or posedge reset) begin
    if (reset)
        select <= 1'b0;
    else begin
        // Detectar flanco de subida
        if (btnU && !btnU_d)
            select <= ~select;
        btnU_d <= btnU;  // Guardar el valor actual para comparar después
    end
end

    
//Instancia Clock
clk_psc clk_scl (
        .clk(clk),
        .clk_scaled(clk2));

// Instancia Moore
FSM_Moore moore (
    .clk(clk),
    .clk2(clk2), 
    .select(select),
    .reset(reset), 
    .D(sw[0]), 
    .C(c_out),     
    .L(L_out),
    .enabled(an),
    .ag(seg)      
);

// Instancia Mealy
FSM_Mealy mealy (
    .clk(clk2), 
    .reset(reset), 
    .ST(sw[1]), 
    .L(L_out), 
    .A(led), 
    .C(c_out)      
);

    always_comb begin
        case(L_out)
            2'b00: l = Digits_0;
            2'b01: l = Digit_1;
            2'b10: l = Digits_2;
            2'b11: l = Digits_3;
            default: l = Digits_0;
        endcase
    end
    
     always_comb begin
        case(led)
            2'b0: A_t = Cerrar;
            2'b1: A_t = Abrir;
            default: A_t = Cerrar;
        endcase
    end
    
    assign dp = 1'b1; //apago el dot

endmodule
