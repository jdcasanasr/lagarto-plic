//**********************************************************************//
//  Proyecto:       SoC_AXI_Lagarto_I                                   //
//  Archivo:        plic_cell.v                                         //
//  Organización:   Instituto Politécnico Nacional                      //
//  Autor(es):      Daniel Casañas                                      //
//  Supervisor:     Dr. Marco Antonio Ramírez Salinas                   //
//  E-mail:         lagarto@cic.ipn.mx                                  //
//  Referencias:    https://github.com/riscv/riscv-plic-spec            //
//**********************************************************************//

`define NO_INTERRUPT 32'b0

module plic_cell
(
    input               interrupt_pending_i,
    input               interrupt_enable_i,

    input       [31:0]  interrupt_source_a_priority_i,
    input       [31:0]  interrupt_source_a_id_i,

    input       [31:0]  interrupt_source_b_priority_i,
    input       [31:0]  interrupt_source_b_id_i,

    output reg  [31:0]  interrupt_source_maximum_priority_o,
    output reg  [31:0]  interrupt_source_maximum_priority_id_o
);

    always @ (*)
        if (interrupt_enable_i && interrupt_pending_i)
            if (interrupt_source_a_priority_i > interrupt_source_b_priority_i)
                begin   
                    interrupt_source_maximum_priority_o     = interrupt_source_a_priority_i;
                    interrupt_source_maximum_priority_id_o  = interrupt_source_a_id_i;
                end
            
            else if (interrupt_source_a_priority_i < interrupt_source_b_priority_i)
                begin
                    interrupt_source_maximum_priority_o     = interrupt_source_b_priority_i;
                    interrupt_source_maximum_priority_id_o  = interrupt_source_b_id_i;
                end
            
            // Break ties with the greatest ID.
            else if (interrupt_source_a_id_i > interrupt_source_b_id_i)
                begin
                    interrupt_source_maximum_priority_o     = interrupt_source_a_priority_i;
                    interrupt_source_maximum_priority_id_o  = interrupt_source_a_id_i;
                end
            
            else
                begin
                    interrupt_source_maximum_priority_o     = interrupt_source_b_priority_i;
                    interrupt_source_maximum_priority_id_o  = interrupt_source_b_id_i;
                end
                
        else
            begin
                interrupt_source_maximum_priority_o     = `NO_INTERRUPT; 
                interrupt_source_maximum_priority_id_o  = `NO_INTERRUPT;
            end

endmodule