
module RAM_Control(
	input clk,
	input reset,
	
	input wire [11:0]CH0,
	input wire [11:0]CH1,
	input wire [11:0]CH2,
	input wire [11:0]CH3,
	input wire [11:0]CH4,
	input wire [11:0]CH5,
	
	// signal to conect to RAM
	 output reg [15:0]  ram_address,   
    output reg [15:0] ram_data,      
    output reg        ram_wren,      
    input  		[15:0] ram_q,         
    output reg [1:0]  ram_byteenable,
    output reg        ram_chipselect,
    output reg        ram_clken,
	 
	//signal to send via bluetooth
	 output reg [47:0] x_coord, //x coord of detected sound
	 output reg [47:0] y_coord, //y coord of detected sound
	 output reg [47:0] z_coord, //z coord of detected sound
	 output wire Rst_n
	);
	
parameter s0=0,s1=1,s2=2,s3=3,s4=4,s5=5,s6=6,s7=7,s8=8,s9=9,s10=10,s11=11,s12=12,s13=13,s14=14;
reg [3:0]state;
reg [2:0]count=0;
reg [4:0]channel=0;
reg [11:0]srcounter=0; 			//counter that controls the sample rate of the ADC
reg [15:0]buffer_pointer=0;	//inicial position circular buffer pointer

reg signed [15:0]values0[5:0];
reg signed [15:0]values1[5:0];
reg signed [20:0]out    [5:0];
reg signed [15:0]out_ARM[5:0];

//Filter signed parameters regs
reg signed [5:0] p1 = 24;
reg signed [5:0] p2 = 25;

reg [2:0]thr_data;
reg [1:0]index=0;
reg impulsive=0;
reg [15:0] count_aux=0;
reg [15:0]arm_end;
reg [3:0]read_count=0;

reg signed [15:0] Threshold=800;
reg rst_n=1'b1;

always@(posedge clk) begin
			case (state)
			
			s0: begin
				//controls the sample rate of the ADC at 16kHz
				if(srcounter==3124 & channel==0)begin
					srcounter<=0;
					
					values0[0]<={1'b0,CH0,3'b0};
					values0[1]<={1'b0,CH1,3'b0};
					values0[2]<={1'b0,CH2,3'b0};
					values0[3]<={1'b0,CH3,3'b0};
					values0[4]<={1'b0,CH4,3'b0};
					values0[5]<={1'b0,CH5,3'b0};
					
					if(buffer_pointer < 7999)
						buffer_pointer <= buffer_pointer+1'b1;
					else
						buffer_pointer <= 0;

					
					
					if (impulsive) begin
						if(count_aux < 7500) begin
							count_aux <= count_aux+1'b1;
							state<=s1;
						end
						else begin
							count_aux <= 0;
							read_count <= 0;
							impulsive <= 1'b0;
							state<=s6;
						end
					end
					else
						state<=s1;
						
				end
				
				else if(channel == 0)
					srcounter <= srcounter+1'b1;
				
				else begin
					srcounter <= srcounter+1'b1;
					state<=s1;
				end
				
			end
			
			s1: begin
				rst_n <= 1'b1;
				ram_wren <= 1'b0;
				ram_chipselect <= 1'b1;
				ram_clken <= 1'b1;	
				ram_byteenable <= 2'b11;
				
				srcounter <= srcounter+1'b1;
				
				case(channel)
					0:ram_address <= 16'h2+buffer_pointer;
					1:ram_address <= 16'h1F42+buffer_pointer;
					2:ram_address <= 16'h3E82+buffer_pointer;
					3:ram_address <= 16'h5DC2+buffer_pointer;
					4:ram_address <= 16'h7D02+buffer_pointer;
					5:ram_address <= 16'h9C42+buffer_pointer;
					6:ram_address <= 16'h1;
					7:ram_address <= 16'h0;
					8:ram_address <= 16'hBB82;
				endcase
				state<=s2;
			end

			s2: begin
			//wait and filter state
				srcounter <= srcounter+1'b1;
				if (count==2) begin
				//simple high pass first order digital filter with 100Hz cut frequency	
					case(channel)

						0:out[0] <= (values0[0]-values1[0]+out_ARM[0])*(p1/p2);
						1:out[1] <= (values0[1]-values1[1]+out_ARM[1])*(p1/p2);
						2:out[2] <= (values0[2]-values1[2]+out_ARM[2])*(p1/p2);
						3:out[3] <= (values0[3]-values1[3]+out_ARM[3])*(p1/p2);
						4:out[4] <= (values0[4]-values1[4]+out_ARM[4])*(p1/p2);
						5:out[5] <= (values0[5]-values1[5]+out_ARM[5])*(p1/p2);
					
					endcase
					count<=0;
					state<=s3;
				end
				else begin
					count <= count+1;
				end
			end
			
			s3: begin
				ram_wren = 1'b1;
				srcounter <= srcounter+1'b1;				

				case(channel)
					0:begin 
						ram_data <= {out[0][20],out[0][14:0]};
						out_ARM[0] <= {out[0][20],out[0][14:0]};
						
						values1[0] <= values0[0];
						if((out[0]>=Threshold) || (out[0]<=-Threshold))
							thr_data[index] <= 1'b1;
						else
							thr_data[index] <= 1'b0;
					end	
					1:begin
						ram_data <= {out[1][20],out[1][14:0]};
						out_ARM[1] <= {out[1][20],out[1][14:0]};
						values1[1] <= values0[1];
					end
					2:begin
						ram_data <= {out[2][20],out[2][14:0]};
						out_ARM[2] <= {out[2][20],out[2][14:0]};
						values1[2] <= values0[2];
					end
					3:begin
						ram_data <= {out[3][20],out[3][14:0]};
						out_ARM[3] <= {out[3][20],out[3][14:0]};
						values1[3] <= values0[3];
					end
					4:begin
						ram_data <= {out[4][20],out[4][14:0]};
						out_ARM[4] <= {out[4][20],out[4][14:0]};
						values1[4] <= values0[4];
					end
					5:begin
						ram_data <= {out[5][20],out[5][14:0]};
						out_ARM[5] <= {out[5][20],out[5][14:0]};
						values1[5] <= values0[5];
					end
					6:ram_data <= buffer_pointer;
					7:ram_data <= 16'b0;
					8:ram_data <= 16'b0;
				endcase
				state <= s4;
			end
			s4: begin
				srcounter <= srcounter+1'b1;
				if (count==2) begin
				
					if(channel==0)begin
					
						if(index<2)
							index<=index+1'b1;
						else
							index<=0;
							
						if(thr_data[0] & thr_data[1] & thr_data[2])
							impulsive <= 1'b1;
					end	
						
					count<=0;
					state<=s5;
				end
				else begin
					count <= count+1;
				end
			end
			s5: begin
				ram_wren <= 1'b0;
				ram_chipselect <= 1'b0;
				ram_clken <= 1'b0;	
				ram_byteenable <= 2'b00;
				
				if(channel==8)
					channel <= 0;
				else
					channel <= channel+1'b1;
					
				srcounter <= srcounter+1'b1;
				
				state <= s0;
			end
			
			s6: begin
				ram_wren <= 1'b0;
				ram_chipselect <= 1'b1;
				ram_clken <= 1'b1;	
				ram_byteenable <= 2'b11;
				ram_address <= 16'h0;
				state<=s7;
			end
			s7: begin
				if (count==2) begin
					count<=0;
					state<=s8;
				end
				else 
					count <= count+1;
			end
			s8: begin
				ram_wren = 1'b1;
				ram_data <= 16'hFFFF;
				state<=s9;
			end
			
			s9: begin
				if (count==2) begin
					count<=0;
					state<=s10;
				end
				else 
					count <= count+1;
			end
			
			s10: begin
				ram_wren <= 1'b0;
				ram_chipselect <= 1'b0;
				ram_clken <= 1'b0;	
				ram_byteenable <= 2'b00;
				state <= s11;
			end
			
			s11: begin
				ram_wren <= 1'b0;
				ram_chipselect <= 1'b1;
				ram_clken <= 1'b1;	
				ram_byteenable <= 2'b11;
				case(read_count)
					 0:ram_address <= 16'hBB82;//arm_end
					 1:ram_address <= 16'hBB83;//x_coord[47:32]
					 2:ram_address <= 16'hBB84;//x_coord[31:16]
					 3:ram_address <= 16'hBB85;//x_coord[15:0]
					 4:ram_address <= 16'hBB86;//y_coord[47:32]
					 5:ram_address <= 16'hBB87;//y_coord[31:16]
					 6:ram_address <= 16'hBB88;//y_coord[15:0]
					 7:ram_address <= 16'hBB89;//z_coord[47:32]
					 8:ram_address <= 16'hBB8A;//z_coord[31:16]
					 9:ram_address <= 16'hBB8B;//z_coord[15:0]
					10:ram_address <= 16'hBB8C;//new Threshold
					//7:ram_address <= 16'hBB89;//x_coord[31:16]
				endcase
				state<=s12;
			end
			
			s12: begin
				if (count==2) begin
					count<=0;
					state<=s13;
				end
				else 
					count <= count+1;
			end
			
			s13: begin
					case(read_count)
					 	 0:arm_end          <= ram_q;//arm_end
					 	 1:x_coord [47:32]  <= ram_q;//x_coord[47:32]
						 2:x_coord [31:16]  <= ram_q;//x_coord[31:16]
						 3:x_coord  [15:0]  <= ram_q;//x_coord[15:0]
						 4:y_coord [47:32]  <= ram_q;//y_coord[47:32]
						 5:y_coord [31:16]  <= ram_q;//y_coord[31:16]
						 6:y_coord  [15:0]  <= ram_q;//y_coord[15:0]
						 7:z_coord [47:32]  <= ram_q;//z_coord[47:32]
						 8:z_coord [31:16]  <= ram_q;//z_coord[31:16]
						 9:z_coord  [15:0]  <= ram_q;//z_coord[15:0]
						10:Threshold [15:0]<= ram_q;//new Threshold
						//7:ram_address <= 16'hBB89;//x_coord[31:16]
				   endcase
					state<=s14;
			end
			
			s14: begin
				ram_wren <= 1'b0;
				ram_chipselect <= 1'b0;
				ram_clken <= 1'b0;	
				ram_byteenable <= 2'b00;
				
				if(arm_end!=0) begin
					if(read_count<10) begin
						read_count <= read_count+1;
						state<=s11;
					end
					else begin
						if(arm_end==2)
							rst_n <= 1'b0;
						count_aux <= 0;
						thr_data<=3'b000;
						impulsive <= 1'b0;
						read_count <= 0;
						state<=s0;
					end
				end
				else begin
					state<=s11;
				end
			end
			endcase
end

assign Rst_n=rst_n;

endmodule