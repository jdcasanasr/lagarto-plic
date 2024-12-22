//**********************************************************************//
//  Proyecto:       SoC_AXI_Lagarto_I                                   //
//  Archivo:        lagarto_plic_multiplexer.v                          //
//  Organización:   Instituto Politécnico Nacional                      //
//  Autor(es):      Daniel Casañas                                      //
//  Supervisor:     Dr. Marco Antonio Ramírez Salinas                   //
//  E-mail:         lagarto@cic.ipn.mx                                  //
//  Referencias:    https://github.com/riscv/riscv-plic-spec            //
//**********************************************************************//

import lagarto_plic_pkg     :: * ;
import riscv_privileged_pkg :: MXLEN;

module lagarto_plic_multiplexer
(
    input   logic                   source_interrupt_pending_i,
    input   logic                   interrupt_enable_i,

    input   interrupt_priority_t    interrupt_priority_a_i,
    input   interrupt_id_t          interrupt_id_a_i,

    input   interrupt_priority_t    interrupt_priority_b_i,
    input   interrupt_id_t          interrupt_id_b_i,

    output  interrupt_priority_t    maximum_priority_o,
    output  interrupt_id_t          maximum_id_o
);

    always_comb
        if (interrupt_enable_i && source_interrupt_pending_i)
            if (interrupt_priority_a_i > interrupt_priority_b_i)
                begin
                    maximum_priority_o  = interrupt_priority_a_i;
                    maximum_id_o        = interrupt_id_a_i;
                end
            
            else if (interrupt_priority_a_i < interrupt_priority_b_i)
                begin
                    maximum_priority_o  = interrupt_priority_b_i;
                    maximum_id_o        = interrupt_id_b_i;
                end
            
            // Break Ties With The Greatest ID.
            else if (interrupt_id_a_i > interrupt_id_b_i)
                begin
                    maximum_priority_o  = interrupt_priority_a_i;
                    maximum_id_o        = interrupt_id_a_i;
                end
            
            else
                begin
                    maximum_priority_o  = interrupt_priority_b_i;
                    maximum_id_o        = interrupt_id_b_i;
                end
                
        else
            begin
                maximum_priority_o  = NO_INTERRUPT_PRIORITY;
                maximum_id_o        = NO_INTERRUPT_ID;
            end

endmodule