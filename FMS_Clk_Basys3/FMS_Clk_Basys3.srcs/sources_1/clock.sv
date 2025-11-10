`timescale 100ns / 1ps

module clock(
    input logic clk, reset,
    output logic [3:0] hora_d, hora_u, min_d, min_u
);

    logic [25:0] contador = 0;
    logic [5:0] segundos = 0;
    logic [3:0] mins_u = 0;
    logic [3:0] mins_d = 0;
    logic [3:0] horas_u = 0;
    logic [3:0] horas_d = 0;

    // Lógica sincronizada al clock principal
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            contador <= 0;
            segundos <= 0;
            mins_u   <= 0;
            mins_d   <= 0;
            horas_u  <= 0;
            horas_d  <= 0;
        end 
        else begin
            // Generador de tick de 1 segundo
            if (contador == 100000 - 1) begin
                contador <= 0;
                segundos <= segundos + 1;
            end 
            else begin
                contador <= contador + 1;
            end

            // Lógica del reloj (todo en el mismo bloque)
            if (segundos == 60) begin
                segundos <= 0;
                mins_u <= mins_u + 1;

                if (mins_u == 9) begin
                    mins_u <= 0;
                    mins_d <= mins_d + 1;

                    if (mins_d == 5) begin
                        mins_d  <= 0;
                        horas_u <= horas_u + 1;

                        if (horas_u == 9) begin
                            horas_u <= 0;
                            horas_d <= horas_d + 1;
                        end

                        if (horas_u == 3 && horas_d == 2) begin
                            horas_u <= 0;
                            horas_d <= 0;
                        end
                    end
                end
            end
        end
    end

    // Salidas
    assign min_u  = mins_u;
    assign min_d  = mins_d;
    assign hora_u = horas_u;
    assign hora_d = horas_d;

endmodule