`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:  Portland State University 
// Engineer: Deepen Parmar
// 
// Create Date: 04/18/2020 02:12:56 AM
// Design Name: 
// Module Name: PWM_Detection_Green
// Project Name: Project-1

// Descript: This function implements the Pulse Widthe Detection logic for the Blue signal
// 

// 
// Revision:
// Revision 0.01 - File Created
// 
// 
//////////////////////////////////////////////////////////////////////////////////

module PWM_Detection_Blue( input clock,reset,
                            input rgb1_blue,
                            output reg [7:0]blue_dutycycle );
reg buf_rgb1_blue;                          
reg [31:0]  buf1_blue_off;
reg [31:0]  buf2_blue_on;
reg [31:0]  buf1_blue;
reg [31:0]  buf2_blue;

reg [31:0]blue_on_temp;
reg [31:0]blue_off_temp;  


int counter_blue_on;
int counter_blue_off; 

    
// Implemening the Synchronizer to meet the Timing Specification
always_ff @(posedge clock)
begin
buf_rgb1_blue <= rgb1_blue;
end

// This always block Check the level of the signal whose Duty cycle is to be measured
always_ff @(posedge clock )
begin
if(~reset)
begin
counter_blue_on <= 'b0;
counter_blue_off <= 'b0;
blue_dutycycle <= 8'b0;
end
else
begin
    begin
    
        if(buf_rgb1_blue)
        begin
        counter_blue_on++;
        counter_blue_off = 'b0;
        end
        else
        begin
        counter_blue_on = 'b0;
        counter_blue_off++;
        end
    end  
end
end     

// This always block Check the level of the signal whose Duty cycle is to be measured
 always_ff @( posedge buf_rgb1_blue)
 begin
 buf1_blue  <= counter_blue_off;
 buf1_blue_off <= buf1_blue;
 end      
 
// Storing the Off counter value  
always_ff @(negedge buf_rgb1_blue)
begin
buf2_blue  <= counter_blue_on;
buf2_blue_on <= buf2_blue;																																								// Counter_blue_On_Temp = Counter_blue_On;
end																																								// Counter_blue_On = 1'b0;//Counter_blue_On_Temp = Counter_blue_On;

// Buffering the values																																							// Counter_blue_Off ++ ;
always_ff @(posedge clock)
 begin
 blue_off_temp <= buf1_blue_off;
  blue_on_temp <= buf2_blue_on;
 end
 
              
                            
 // Calculating the duty cycle                         
 assign blue_dutycycle = (2*100*blue_on_temp) / (blue_on_temp + blue_off_temp);                           
                                   
                            
                            

endmodule
