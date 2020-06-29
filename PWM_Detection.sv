`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Portland State University 
// Engineer: Deepen Parmar
// 
// Create Date: 04/18/2020 02:12:56 AM
// Design Name: 
// Module Name: PWM_Detection
// Project Name: Project-1

// Descript: This module Instantiated the three module and connects withthe top level 
// 

// 
// Revision:
// Revision 0.01 - File Created
// 
// 
//////////////////////////////////////////////////////////////////////////////////

module PWM_Detection(

input clock,reset,
input RGB1_Red,
input RGB1_Blue,
input RGB1_Green,
output reg [7:0] Red_Dutycycle,
output reg [7:0]Blue_Dutycycle,
output reg [7:0]Green_Dutycycle
 );
 
PWM_Detection_Red  PWM_Detection_Red ( .clock(clock),
                                       .reset(reset),
                                       .rgb1_red(RGB1_Red),
                                       .red_dutycycle(Red_Dutycycle) 
);

PWM_Detection_Green  PWM_Detection_Green ( .clock(clock),
                                           .reset(reset),
                                        .rgb1_green(RGB1_Green),
                                      .green_dutycycle(Green_Dutycycle) 
);

PWM_Detection_Blue  PWM_Detection_Blue ( .clock(clock),
                                         .reset(reset),
                                         .rgb1_blue(RGB1_Blue),
                                         .blue_dutycycle(Blue_Dutycycle) 
);


 
 
endmodule

