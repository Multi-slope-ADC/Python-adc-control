# ADC control program (Python)<br>
Python clone of [adc control program (free pascal) from Ulrich Harms](https://www.eevblog.com/forum/metrology/diy-high-resolution-multi-slope-converter/msg3616117/#msg3616117)<br>
<br>
Changelog:<br>
Logfiles: Values with tab separation<br>
Mode B (INL test): improved interpolation<br>
Added set scalefactor for mode A & E - new function Z<br>
Added update for k1 & k2 values for function "L"<br>
<br>
[Clone w/o changes is maintained in branch: Clone_of_Pascal](https://github.com/Multi-slope-ADC/Python-adc-control/tree/Clone_of_Pascal)<br>
<br>
## Manual<br>
Commands are sent as 1 byte ASCII:<br>
<br>
0..7 Selection of MUX channel for measurement (Input1..8 on U7 DG408)<br>
&nbsp;0 -> Pin 7 on connector J2<br>
&nbsp;1 -> Pin 5 on connector J2<br>
&nbsp;2 -> Pin 3 on connector J2<br>
&nbsp;3 -> Pin 1 on connector J2<br>
&nbsp;4 -> temperature diode<br>
&nbsp;5 -> Pin 9 on connector J2 - connected to frontend<br>
&nbsp;6 -> 7V Ref without buffer<br>
&nbsp;7 -> 0V (GNDS)<br>
<br>
A..E reading mode:<br>
&nbsp;A -> 2 readings AZ (S 0) - needs scalefactor correction: let it run for some time (m_sf #readings), switch to function L (console output: update scalefactor from ref reading), then back to mode A<br>
&nbsp;B -> INL test difference 2 run-up modes 1. run-up Q, 2. selected (S S)<br>
&nbsp;C -> 3 readings (S 0 7)<br>
&nbsp;D -> 4 readings (S S2 (channel 2) 0 7 - 2 channels + ref)<br>
&nbsp;E -> 2 readings (S S2 (channel 2) - difference to channel 2)<br>
<br>
F..M functions:<br>
&nbsp;F -> reset integration time to 1 PLC (possibly side effect)<br>
&nbsp;G -> integration capacitor DA test (not implemented in control program)<br>
&nbsp;I -> infinite run-up (for test - only continue with reset?)<br>
&nbsp;K -> old version slope measurement (stops if necessary)<br>
&nbsp;L -> slope measurement K1 and K2<br>
&nbsp;M -> double integration time (2, 4, 8, ... PLC)<br>
&nbsp;Z -> set scalefactor for mode A & E<br>
<br>
P..W run-up versions:<br>
&nbsp;P -> fast 35+3\*x/8/8<br>
&nbsp;Q -> normal case 78+6\*x/12/12<br>
&nbsp;R -> short fix 86+6\*x/8/8<br>
&nbsp;S -> 4 step mode<br>
&nbsp;T -> dummy 4 step mode (integrator MUX U5 4053 always off)<br>
&nbsp;U -> 4 step with 0<br>
&nbsp;V -> long fix 66+6\*x/18/18<br>
&nbsp;W -> slow 168+12\*x/18/18<br>
<br>
X -> exit control program (no changes in adc)<br>
<br>
Raw data format received from ADC:<br>
0xFF sync<br>
1 byte selection mode / type of data, for sync generally> 128<br>
16 bit run-up counts<br>
16 bit cycles for pos ref, step size is 4<br>
16 bit cycles for neg ref, step size is 4<br>
16 bit aux ADC reading (average voltage at the integrator output)<br>
16 bit residual ADC reading after conversion<br>
16 bit residual ADC reading before conversion<br>
possibly more data records (without sync and mode) one after the other, depending on the number of conversions in the cycle<br>
<br>
<br>
The factors K1 and K2 are used to calculate the result.<br>
K1 is the ratio of the negative reference and the sum (- Vref- / (Vref+ + Vref-)).<br>
K2 gives the scale factor for the µC internal ADC.<br>
<br>
The two quantities are measured by sending the command "L" to the ADC.<br>
The appropriate calibration measurements are then made.<br>
The values ​​also end up in the file and are directly adopted for the measurement, but they can be manually set in the program.<br>
The notation as 1 / ... and 4 / ... and comes from the fact that the outputs from the program just correspond to the ... values.<br>
<br>
At the console the 5th value is the voltage in mV, the 6th value is the raw value of the µC internal ADC.<br>
This value is influenced by the trimmer and should be around the middle of the range, i.e. around 300-700, and not come to the limit.<br>
That the values ​​fluctuate is normal, these are just the different measured values ​​corresponding to the lower 6-8 bits.<br>
The trimmer isn't particularly sensitive - the range for the ADC should be around 2-4 revolutions.<br>
The setting should be suitable for the calibration measurement to work.<br>
To see directly what the trimmer is doing, you can look at the signal at the output of the NE5534, that should shift.<br>
<br>
Normally the conversions are 20 ms = 1 PLC @ 50Hz mains (not particularly precise - something could possibly be adjusted).<br>
The time can be doubled using the "M" command (i.e. 2, 4, 8 PLC) and set back to 1 with "F".<br>
The outputs on the screen are mean values ​​over more conversions (e.g. 20 one channel constant at the top of the program).<br>
In the file, the values ​​are generally 1 PLC and due to the auto zero mode 1 value approximately every 40.4 ms (mode B, longer with 3 or 4 values in mode C/D).<br>
<br>
