`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Portland State University 
// Engineer: Deepen Parmar
// 
// Create Date: 04/18/2020 02:12:56 AM
// Design Name: 
// Module Name: PWM_Detection_Green
// Project Name: Project-1

// Descript: This function implements the Pulse Widthe Detection logic for the Green signal
// 

// 
// Revision:
// Revision 0.01 - File Created
// 
// 
//////////////////////////////////////////////////////////////////////////////////


module PWM_Detection_Green( input clock,reset,
                            input rgb1_green,
                            output reg [7:0]green_dutycycle );

reg buf_rgb1_green;                          
reg [31:0]  buf1_green_off;
reg [31:0]  buf2_green_on;
reg [31:0]  buf1_green;
reg [31:0]  buf2_green;

reg [31:0]green_on_temp;
reg [31:0]green_off_temp;  


int counter_green_on;
int counter_green_off;  
reg clk_high;   
reg clk_low;


// Implemening the Synchronizer to meet the Timing Specification
always_ff @(posedge clock)
begin
buf_rgb1_green <= rgb1_green;
end
                        
// This always block Check the level of the signal whose Duty cycle is to be measured
always_ff @(posedge clock )
begin
if(~reset)
begin
counter_green_on <= 'b0;
counter_green_off <= 'b0;
green_dutycycle <= 8'b0;
end
else
begin

    if(buf_rgb1_green)
    begin
    counter_green_on++;
    counter_green_off = 'b0;
    end
    else
    begin
    counter_green_on = 'b0;
    counter_green_off++;
    end
end
end   

// This always block Check the level of the signal whose Duty cycle is to be measured
 always_ff @( posedge buf_rgb1_green)
 begin
 buf1_green  <= counter_green_off;
 buf1_green_off <= buf1_green;
 end      
 
 
// Storing the Off counter value 
always_ff @(negedge buf_rgb1_green)
begin
buf2_green  <= counter_green_on;
buf2_green_on <= buf2_green;																																								// Counter_green_On_Temp = Counter_green_On;
end																																								// Counter_green_On = 1'b0;//Counter_green_On_Temp = Counter_green_On;
																																								// Counter_green_Off ++ ;
always_ff @(posedge clock)
 begin
 green_off_temp <= buf1_green_off;
  green_on_temp <= buf2_green_on;
 end
                    
 // Calculating the Duty Cycle value                    
 assign green_dutycycle = (2*100*green_on_temp) / (green_on_temp + green_off_temp);              
                            
                                                      
                            
                            

endmodule
