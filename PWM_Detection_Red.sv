`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Portland State University 
// Engineer: Deepen Parmar
// 
// Create Date: 04/18/2020 02:12:56 AM
// Design Name: 
// Module Name: PWM_Detection_Red
// Project Name: Project-1

// Descript: This function implements the Pulse Widthe Detection logic for the Red signal
// 

// 
// Revision:
// Revision 0.01 - File Created
// 
// 
//////////////////////////////////////////////////////////////////////////////////


module PWM_Detection_Red( input clock,reset,
                            input rgb1_red,
                            output reg [7:0] red_dutycycle
                            );
reg buf_rgb1_red;     
reg buf_rgb2_red;                        
                            
reg [31:0]  buf1_red_off;
reg [31:0]  buf2_red_off;

reg [31:0]  buf2_red_on;
reg [31:0]  buf2_red1_on;

reg [31:0]  buf1_red;
reg [31:0]  buf2_red;

reg [31:0]red_on_temp;
reg [31:0]red_off_temp; 

reg [1:0]in_buf_state; 


int  counter_red_on;
int  counter_red_off;

                    
// Implemening the Synchronizer to meet the Timing Specification
always_ff @(posedge clock)
begin
buf_rgb1_red <= rgb1_red;
end

// This always block Check the level of the signal whose Duty cycle is to be measured
always_ff @(posedge clock )
begin
if(~reset)
begin
counter_red_on <= 'b0;
counter_red_off <= 'b0;
red_dutycycle <= 8'b0;
end
else
begin

    if(buf_rgb1_red)
    begin
    counter_red_on++;
    counter_red_off = 'b0;
    end
    else
    begin
    counter_red_on = 'b0;
    counter_red_off++;
    end
end
end   

//  Measures the duty cycle at the posedge of the signal when 1 cycle is completed
 always_ff @( posedge buf_rgb1_red)
 begin
 buf1_red  <= counter_red_off;
 buf1_red_off <= buf1_red;
 red_dutycycle <= (2*100*red_on_temp) / (red_on_temp + red_off_temp);
 end      
 
 
// Stores the off counter value in the regiter
always_ff @(negedge buf_rgb1_red)
begin
buf2_red  <= counter_red_on;
buf2_red_on <= buf2_red;																																								
end																																								


// Buffers the value																																						
always_ff @(posedge clock)
 begin
 buf2_red_off <= buf1_red_off;
 red_off_temp <= buf2_red_off;
  buf2_red1_on <= buf2_red_on;
red_on_temp <= buf2_red1_on;
 end
 
endmodule



















