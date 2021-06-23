# aoa_locator
=========== create simulation I Q data ============== 
modul  Simulator_I_Q reference as p 3 Interpreting IQ Samples of AN1297: "Custom Direction-Finding Solutions using the Silicon Labs Bluetooth Stack"
file Simulator_I_Q.c
created samples for reference period as p 3.4 "Phase compensation"
next created samples for snapshots vs given settings (on str. 18)

=========== file Sample.csv ===============
created in the function ( get_samples(aoa_iq_report_t *iq_report,float fr) ) in the file aoa.c (296)
parse the given aoa_iq_report_t (were samples may be simulation data)
CSV settings: values with separators - ';', decimal separated - '.'(point)

=========== file IQ_Report_data_log.csv ===============
created in Bluetooth event handler (void app_bt_on_event(sl_bt_msg_t *evt)) in app_silabs.c file(196) using method I_Q_to_CSV()
-calculate phase on each path of antenna
-calculate own phase shift on each path through each snapshot
-calculate amplitude of sygnal on this phase
-calculate the phase difference between 0 to 1, 1 to 2, 2 to 3 path of antenna
CSV settings: Separated values - ';', decimal separated - '.'(point)
