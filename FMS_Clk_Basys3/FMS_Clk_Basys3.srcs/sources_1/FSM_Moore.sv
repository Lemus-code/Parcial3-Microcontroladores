`timescale 100ns / 1ps

module FSM_Moore(
    input  logic clk, clk2, reset, D, C,      // D = dígito leído , C = lectura completada (viene de mealy)  
    input logic select,
    output logic [1:0] L,                     // L = longitud código/sticker
    output logic [3:0] enabled,
    output logic [6:0] ag            
);

//Plantilla de estados
typedef enum logic [1:0] {S0,S1,S2,S3} statetype;
statetype state, nextstate;

//Plantilla de L
typedef enum logic [1:0] {Digits_0, Digit_1, Digits_2, Digits_3} outtype;
outtype l;

//Señales internas
logic [3:0] dread = 4'b0000;                //Conteo digits
logic [3:0] hora_d, hora_u, min_d, min_u;   //Reloj local
logic [3:0] d0, d1, d2, d3;                 //datos display

// Registro de estados
always_ff @(posedge clk2 or posedge reset)
    if (reset) begin state <= S0; dread <= 0; end
    else 
        begin
            state <= nextstate;
            //Contar dígitos
            if(D && ~C) 
                begin
                    if(dread < 3)dread <= dread + 1;
                    else dread <= dread;
                end
            else if(~D && C) dread  <= 0;
            else dread  <= dread;
    end

// Next State Logic
always_comb
    case (state)    
        S0: if (D && ~C) nextstate = S1;
            else nextstate = S0;

        S1: if (~D && C) nextstate = S0;
            else if (D && ~C) nextstate = S2;
            else nextstate = S1;

        S2: if (~D && C) nextstate = S0;
            else if (D && ~C) nextstate = S3;
            else nextstate = S2;

        S3: if (~D && C) nextstate = S0;
            else nextstate = S3; 

        default: nextstate = S0;
    endcase

// Output Logic
always_comb 
    case (state)
        S0: l = Digits_0;
        S1: l = Digit_1;
        S2: l = Digits_2;
        S3: l = Digits_3;
    endcase

assign L = l;

//Llevar conteo del clk interno
clock reloj_local(
        .clk(clk),
        .reset(reset),
        .hora_d(hora_d),
        .hora_u(hora_u),
        .min_d(min_d),
        .min_u(min_u)
    );
    
//Alternar entre clk local y data vivepass
always_comb 
    begin
        if(select == 0)
            begin
                d0 = dread;
                d1 = 4'b0000;
                d2 = 4'b0000;
                d3 = 4'b0000;
            end
        else
            begin
                d0 = min_u;
                d1 = min_d;
                d2 = hora_u;
                d3 = hora_d;   
            end
    end
 
 //Datos display       
display_7segments conteo(
    .clk(clk),
    .d0(d0),
    .d1(d1),
    .d2(d2),
    .d3(d3),
    .enabled(enabled),
    .ag(ag)
);

endmodule
