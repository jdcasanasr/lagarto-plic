module lagarto_plic_gateway_tb.sv ();

    logic interrupt_signal_r;
    logic interrupt_claim_complete_r;

    logic interrupt_request_w;

    always
        #20 interrupt_signal_r = $random(42);

    initial
        begin
            #30 interrupt_claim_complete_r = '1;
            #50 interrupt_claim_complete_r = '0;

            #20 interrupt_claim_complete_r = '1;
            #50 interrupt_claim_complete_r = '0;

            #20 ;
            #20 $stop;
        end

    lagarto_plic_gateway gateway_instance
    (
        .interrupt_signal_i         (interrupt_signal_r),
        .interrupt_claim_complete_i (interrupt_claim_complete_r),

        .interrupt_request_o        (interrupt_request_w)
    );

endmodule