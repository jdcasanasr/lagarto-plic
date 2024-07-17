//**********************************************************************//
//  Proyecto:       SoC_AXI_Lagarto_I                                   //
//  Archivo:        top.sv                                              //
//  Organización:   Instituto Politécnico Nacional                      //
//  Autor(es):      Daniel Casañas                                      //
//  Supervisor:     Dr. Marco Antonio Ramírez Salinas                   //
//  E-mail:         lagarto@cic.ipn.mx                                  //
//  Referencias:    https://github.com/riscv/riscv-plic-spec            //
//**********************************************************************//

plic_core plic_core_instance
(
    // Source-side signals.
    .jtag_signal    (),
    .gpio0_signal   (),
    .gpio1_signal   (),
    .gpio2_signal   (),
    .spi0_signal    (),
    .spi1_signal    (),

);