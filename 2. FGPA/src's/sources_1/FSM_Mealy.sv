`timescale 100ns / 1ps

module FSM_Mealy(
    input  logic clk, reset, ST, [1:0] L,   // entrada desde la Moore
    output logic A,C          // A = Cerrar talanquera , C =lectura completa (para Moore)
);

// Estados
typedef enum logic [1:0] {S0, S1} statetype;
statetype current_state, next_state;

// Registro de estado
always_ff @(posedge clk or posedge reset)
    if (reset) current_state <= S0;
    else current_state <= next_state;

// Next State logic
always_comb begin
    next_state = current_state;
    case (current_state)
        S0: if (~ST && L == 2'b11) next_state = S1;   
        S1: if (ST && L == 2'b11) next_state = S0;    
        default: next_state = S0;
    endcase
end

// Output Logic 
always_comb begin
    C = 1'b0;
    A = 1'b0;

    case (current_state)
        S0: begin
            if (~ST && (L == 2'b00 || L == 2'b01 || L == 2'b10 || L == 2'b11)) begin
                C = 1'b0;  A = 1'b0;
            end
            else if (ST && (L == 2'b00 || L == 2'b01 || L == 2'b10)) begin
                C = 1'b1;  A = 1'b0;   // Lectura válida pero no abre
                end
                else if (ST && L == 2'b11) begin
                    C = 1'b1;  A = 1'b1;   // Lectura válida y abre
                end
        end

        S1: begin
            if (~ST && L == 2'b11) begin  
                C = 1'b0;  A = 1'b0;
            end
            else if (ST && L == 2'b11) begin 
                C = 1'b1;  A = 1'b1;
            end
        end
        default: begin
            C = 1'b0;
            A = 1'b0;
        end
    endcase
end

endmodule