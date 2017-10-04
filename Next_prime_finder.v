//Jonathan Wapman
//Prime Number Tester
//Tests whether a number is prime. If not, tests the following numbers
//Until a prime is found

module lab7(

	//////////// CLOCK //////////
	input 		          		ADC_CLK_10,
	input 		          		MAX10_CLK1_50,
	input 		          		MAX10_CLK2_50,

	//////////// SEG7 //////////
	output		     [7:0]		HEX0,
	output		     [7:0]		HEX1,
	output		     [7:0]		HEX2,
	output		     [7:0]		HEX3,
	output		     [7:0]		HEX4,
	output		     [7:0]		HEX5,

	//////////// KEY //////////
	input 		     [1:0]		KEY,

	//////////// LED //////////
	output reg		     [9:0]		LEDR,

	//////////// SW //////////
	input 		     [9:0]		SW
);

//=======================================================
//  REG/WIRE declarations
//=======================================================

//Number Generator Value
wire [31:0]  value;


wire rst, sample, start;
wire [1:0] debug;
wire [7:0] is_prime, done;
wire [7:0] write_data_orig, write_data_prime, read_data_orig, read_data_prime;
wire write_enable_orig, write_enable_prime;
wire [1:0] write_addr_orig, write_addr_prime, read_addr_orig, read_addr_prime, read_addr;
wire c0_sig, c1_sig, clk;

//State Machine Variables
reg [31:0] count, count_c;
reg [31:0] orig_val, orig_val_c;
reg [31:0] prime_val, prime_val_c;
reg [2:0] state, state_c;
reg LED_prime, LED_prime_c;
reg LED_done, LED_done_c;
reg startTest; //Makes prime testers start and stop testing
wire [7:0] count_prime;
wire reset_prime_tester;
wire dividend = count;
wire data_sel;
wire key1_start;

//State Machine Parameters
parameter WAIT = 3'b000;
parameter CLK_WAIT_1 = 3'b001;
parameter TEST_ORIGINAL = 3'b010;
parameter CHECK_ORIGINAL = 3'b011;
parameter CLK_WAIT_2 = 3'b100;
parameter CLK_WAIT_3 = 3'b101;
parameter TEST_NEXT = 3'b110;
parameter CHECK_NEXT = 3'b111;


//PLL
pll	pll_inst (
	.inclk0 ( MAX10_CLK1_50 ),
	.c0 ( clk ), //100 MHz
	.c1 ( c1_sig ) //125MHz. Unused
	);

//assign clk = MAX10_CLK1_50; //50 MHz, unused
	
//Synchronizers for inputs
synchronizer s1(.clk(clk), .in(SW[9]), .out(rst)); //Reset
synchronizer s2(.clk(clk), .in(KEY[1]), .out(key1_start)); //Key to start testing the number
synchronizer s3(.clk(clk), .in(KEY[0]), .out(sample)); //Key to generate random number
synchronizer s4(.clk(clk), .in(SW[4]), .out(data_sel)); //Switch to choose whether to display the original number or the next prime number. 0 = original, 1 = prime
synchronizer s5(.clk(clk), .in(SW[3]), .out(read_addr[1])); //Switches to choose which 8-bit address to read from
synchronizer s6(.clk(clk), .in(SW[2]), .out(read_addr[0]));
synchronizer s7(.clk(clk), .in(SW[1]), .out(debug[1])); //Switches to choose which number to test with
synchronizer s8(.clk(clk), .in(SW[0]), .out(debug[0]));


//Number generator
number_generator gen(.clk(clk), .rst(rst), .sample(sample), .debug(debug), .value(value));

//Prime number testers
prime_tester prime0(.clk(clk), .rst(rst), .dividend(count), .start(startTest), .done(done[0]), .is_prime(is_prime[0]), .count(c), .start_val(32'd2), .end_val((count >> 4))); //Divide by 16, check first eigth
prime_tester prime1(.clk(clk), .rst(rst), .dividend(count), .start(startTest), .done(done[1]), .is_prime(is_prime[1]), .count(c), .start_val((count >> 4)), .end_val((count >> 4) * 2'd2));
prime_tester prime2(.clk(clk), .rst(rst), .dividend(count), .start(startTest), .done(done[2]), .is_prime(is_prime[2]), .count(c), .start_val((count >> 4) * 2'd2), .end_val((count >> 4) * 2'd3));
prime_tester prime3(.clk(clk), .rst(rst), .dividend(count), .start(startTest), .done(done[3]), .is_prime(is_prime[3]), .count(c), .start_val((count >> 4) * 2'd3), .end_val((count >> 4) * 3'd4));
prime_tester prime4(.clk(clk), .rst(rst), .dividend(count), .start(startTest), .done(done[4]), .is_prime(is_prime[4]), .count(c), .start_val((count >> 4) * 3'd4), .end_val((count >> 4) * 3'd5));
prime_tester prime5(.clk(clk), .rst(rst), .dividend(count), .start(startTest), .done(done[5]), .is_prime(is_prime[5]), .count(c), .start_val((count >> 4) * 3'd5), .end_val((count >> 4) * 3'd6));
prime_tester prime6(.clk(clk), .rst(rst), .dividend(count), .start(startTest), .done(done[6]), .is_prime(is_prime[6]), .count(c), .start_val((count >> 4) * 3'd6), .end_val((count >> 4) * 3'd7));
prime_tester prime7(.clk(clk), .rst(rst), .dividend(count), .start(startTest), .done(done[7]), .is_prime(is_prime[7]), .count(c), .start_val((count >> 4) * 3'd7), .end_val((count >> 1) + 1'b1)); //check up to half + 1

//Falling edge detector for when the search process should start
edge_detector_falling state_detector_falling(.clk(clk), .input_signal(key1_start), .falling_transition(start));


//State Machine used to check whether the original number is prime. If not, tests following numbers one by one until a prime number is found
always @(*) begin
	//Defaults
	count_c = count; //Store the current count
	state_c = state; //Store the current state
	LED_prime_c = LED_prime; //Store whether the Prime LED should be on or off
	LED_done_c = LED_done; //Stores whether the Done LED should be on or off
	orig_val_c = orig_val; //Stores the original value
	prime_val_c = prime_val; //Stores the first found prime value
	
	LEDR[1] = LED_done;
	LEDR[0] = LED_prime;
	LEDR[9:2] = done; //Displays the state of each prime number tester module
	
	case (state)
	
		WAIT: begin //State 000
			startTest = 1'b1; //Stay high to prevent testing
			
			if (start == 1'b1) begin //Reset to test original number
				state_c = CLK_WAIT_1;
				count_c = value; //Set the starting point to the value given by the number generator
				LED_done_c = 1'b0; //Turn off LEDs
				LED_prime_c = 1'b0;
				startTest = 1'b0; //Go to low and hold to start testing
				orig_val_c = value; //Save the generated value to the original value's position
				prime_val_c = 32'd0; //Reset the prime value
			end
			
		end
		
		CLK_WAIT_1: begin //State 001. Gives prime testers 1 clock cycle to reset
			startTest = 1'b0;
			state_c = TEST_ORIGINAL;
			orig_val_c = value; //Save the generated value to the original value's position
		end
		
		TEST_ORIGINAL: begin //State 010
			startTest = 1'b1;
			if (done == 8'b11111111) begin //If done testing
				state_c = CHECK_ORIGINAL;
				
			end
		end
		
		CHECK_ORIGINAL: begin //State 011. Tests whether the original number is prie
			startTest = 1'b1; //Prevent testing
			if (is_prime == 8'b11111111) begin
				state_c = WAIT; //Go to wait state
				LED_done_c = 1'b1; //Turn on the light indicating the process is done;
				LED_prime_c = 1'b1; //Indicate the number is prime
				prime_val_c = value; //Save the value to the register storing the first prime value
				orig_val_c = value; //Save the generated value to the original value's position
			end
			else begin //If not prime, increment count to test next value
				count_c = count + 32'b1; //Next value to test
				state_c = CLK_WAIT_2;
				orig_val_c = value; //Save the generated value to the original value's position;
			end
		end
		
		CLK_WAIT_2: begin //State 100. Used to give the prime testers time to reset before testing the next value
			startTest = 1'b0;
			state_c = CLK_WAIT_3;
		end
		
		CLK_WAIT_3: begin //State 101. Used to give the prime testers time to reset
			startTest = 1'b0;
			state_c = TEST_NEXT;
		end
		
		TEST_NEXT: begin //State 110. Tests the next number
			state_c = TEST_NEXT;
			startTest = 1'b1;
			if(done == 8'b11111111) begin //If the testing process has finished
				state_c = CHECK_NEXT;
			end
			
		end
		
		CHECK_NEXT: begin //State 111. Tests whether the checked number is prime or not
			startTest = 1'b1; //Pause testing
			if (is_prime == 8'b11111111) begin //If a prime has been found, finish
					state_c = WAIT;
					LED_done_c = 1'b1;
					prime_val_c = count; //Store the iterated value to the prime value register
			end
			else begin //If number is not prime, check next number
				state_c = CLK_WAIT_2;
				count_c = count + 32'b1; //Increment to test next number
				startTest = 1'b0; //Restart the testing process
			end
		
		end
	
	endcase
	
	if (rst == 1'b1) begin //Reset
		count_c = 32'b0;
		state_c = WAIT;
		LED_prime_c = 1'b0; //Turn off LED
		LED_done_c = 1'b0; //Turn off LED
		orig_val_c = 32'b0; //Reset number values
		prime_val_c = 32'b0;
	end

end

//FF instantiation
always @(posedge clk) begin
	state <= #1 state_c;
	count <= #1 count_c;
	LED_done <= #1 LED_done_c;
	LED_prime <= #1 LED_prime_c;
	prime_val <= #1 prime_val_c;
	orig_val <= #1 orig_val_c;
end



//Serializer
serializer ser_orig(.clk(clk), .rst(rst), .save(LED_done), .data_in(orig_val), .write_data(write_data_orig), .write_enable(write_enable_orig), .write_addr(write_addr_orig));
serializer ser_prime(.clk(clk), .rst(rst), .save(LED_done), .data_in(prime_val), .write_data(write_data_prime), .write_enable(write_enable_prime), .write_addr(write_addr_prime));

//RAM
output_ram ram_orig(.clk(clk), .write_data(write_data_orig), .write_enable(write_enable_orig), .write_addr(write_addr_orig), .read_addr(read_addr_orig), .read_data(read_data_orig));
output_ram ram_prime(.clk(clk), .write_data(write_data_prime), .write_enable(write_enable_prime), .write_addr(write_addr_prime), .read_addr(read_addr_prime), .read_data(read_data_prime));

//Byte to SEG7
assign read_addr_orig = read_addr;
assign read_addr_prime = read_addr;
byte2_7seg b_orig(.data_sel(data_sel), .in_orig(read_data_orig), .in_prime(read_data_prime), .hex0out(HEX0), .hex1out(HEX1));
	
//Turn off other 7 Segment displays
assign HEX2 = 8'b11111111;
assign HEX3 = 8'b11111111;
assign HEX4 = 8'b11111111;
assign HEX5 = 8'b11111111;

endmodule

//Byte to 7 seg
module byte2_7seg(data_sel, in_orig, in_prime, hex0out, hex1out);
	input [7:0] in_orig, in_prime; //Input data for the original number and prime number
	output [7:0] hex0out, hex1out; //HEX outputs
	wire [7:0] display; //Wire to store the data for either the original number or the prime number
	input data_sel; //Connects to switch that selects whether to display the data from the original number or the first found prime number

	assign display = data_sel ? in_prime : in_orig; //Mux
	
	hex_2_7_seg m1(.in(display[7:4]), .out(hex1out));
	hex_2_7_seg m2(.in(display[3:0]), .out(hex0out));
	
endmodule

//hex27_seg. Converst a binary number to a HEX display output
module hex_2_7_seg(in, out);
	input [3:0] in;
	output reg [7:0] out;
	
	parameter ZERO =  8'b1100_0000;
	parameter ONE =   8'b1111_1001;
	parameter TWO = 8'b1010_0100;
	parameter THREE = 8'b1011_0000;
	parameter FOUR = 8'b1001_1001;
	parameter FIVE = 8'b1001_0010;
	parameter SIX = 8'b1000_0010;
	parameter SEVEN = 8'b1111_1000;
	parameter EIGHT = 8'b1000_0000;
	parameter NINE = 8'b1001_1000;
	parameter A = 8'b1000_1000;
	parameter B = 8'b1000_0011;
	parameter C = 8'b1100_0110;
	parameter D = 8'b1010_0001;
	parameter E = 8'b1000_0110;
	parameter F = 8'b1000_1110;
	
	always @(*) begin
		case (in)
			0: out = ZERO;
			1: out = ONE;
			2: out = TWO;
			3: out = THREE;
			4: out = FOUR;
			5: out = FIVE;
			6: out = SIX;
			7: out = SEVEN;
			8: out = EIGHT;
			9: out = NINE;
			10: out = A;
			11: out = B;
			12: out = C;
			13: out = D;
			14: out = E;
			15: out = F;
		endcase
	end

endmodule

//RAM. 4 Words, 8 bits each
module output_ram(clk, write_data, write_enable, write_addr, read_addr, read_data);
	input [7:0] write_data;
	input write_enable, clk;
	input [1:0] write_addr;
	input [1:0] read_addr;
	output [7:0] read_data;
	
	reg [7:0] Mem [0:3]; //4 addresses, 8 bits

	//Write
	always @(posedge clk) begin
		if (write_enable == 1'b1) begin
			Mem[write_addr] <= #1 write_data;
		end
	end
	
	assign read_data = Mem[read_addr]; //Read
	
endmodule

//Serializer. Writes data to the RAM. 1 byte at a time over 4 clock cycles
module serializer(clk, rst, save, data_in, write_data, write_enable, write_addr);
	input save, clk, rst;
	input [31:0] data_in;
	output reg [7:0] write_data;
	output reg write_enable;
	output reg [1:0] write_addr;
	wire rising_transition;
	
	//Parameters
	parameter WAIT = 3'b000;
	parameter ADDR1 = 3'b001;
	parameter ADDR2 = 3'b010;
	parameter ADDR3 = 3'b011;
	parameter ADDR4 = 3'b100;
	
	reg [2:0] state, state_c;
	
	edge_detector_rising test(.clk(clk), .input_signal(save), .rising_transition(rising_transition)); //Detects when the save signal transitions from low to high
	
	always @(*) begin
		//Default. Assign to previous state
		state_c = state;
		
		//Write data 8 bits at a time
		case (state)
			WAIT: begin
				write_enable = 1'b0;
				if (rising_transition == 1'b1) begin //If save changes to high
					state_c = ADDR1;
				end
			end
			ADDR1: begin
				write_enable = 1'b1;
				write_addr = 2'b00;
				write_data = data_in[7:0];
				state_c = ADDR2;
			end
			ADDR2: begin
				write_enable = 1'b1;
				write_addr = 2'b01;
				write_data = data_in[15:8];
				state_c = ADDR3;
			end
			ADDR3: begin
				write_enable = 1'b1;
				write_addr = 2'b10;
				write_data = data_in[23:16];
				state_c = ADDR4;
			end
			ADDR4: begin
				write_enable = 1'b1;
				write_addr = 2'b11;
				write_data = data_in[31:24];
				state_c = WAIT;
			end
		endcase
		
		if (rst == 1'b1) begin
			state_c = WAIT;
		end
	end
	
	//FFs
	always @(posedge clk) begin
		state <= #1 state_c;
	end
endmodule

//Synchronizer. Synchronizes inputs with the clock using FFs over 3 clock cycles
module synchronizer(clk, in, out);
	
	input clk, in;
	output reg out;
	reg ff1, ff1_c, ff2, ff2_c, ff3, ff3_c;
	
	always @(*) begin
		ff1_c = in;
		ff2_c = ff1;
		ff3_c = ff2;
		out = ff3;
	end
	
	always @(posedge clk) begin
		ff1 <= #1 ff1_c;
		ff2 <= #1 ff2_c;
		ff3 <= #1 ff3_c;
		
	end
	
endmodule

//Determines whether a number is a prime number
module prime_tester(clk, rst, dividend, start, done, is_prime, count, start_val, end_val, rem0);


	//Inputs
	input [31:0] dividend; //Number to test
	input start, clk, rst;
	input [31:0] start_val, end_val; //To make modular, multiple prime checkers
	//Outputs
	output reg done, is_prime;
	reg done_c, is_prime_c;
	
	//Counter. 1 bit larger than the number being tested to avoid overflow
	output reg [32:0] count;
	reg [32:0] count_c;
	
	wire [32:0] endValConcat = {1'b0, end_val}; //Makes the most significant bit a 0
	wire [32:0] startValConcat = {1'b0, start_val};
	
	//Run conditions
	wire falling_transition;
	reg run, run_c;
	reg clear; //Used to reset remainder checker
	
	//Instantiate falling edge module
	edge_detector_falling edge_detector_falling2(.clk(clk), .input_signal(start), .falling_transition(falling_transition));
	
	//Instantiate remainder testing module
	//output [23:0] remainder_done;
	wire [23:0] remainder_done;
	wire [31:0] rem;
	reg [31:0] remainder;
	wire [31:0] remainder_c;
	output [23:0] rem0;  //Whether the number has a remainder of 0
	
	//Remainder testers. Check every other number to only check the odds
	remainder_test test0(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0]), .done(remainder_done[0]), .rem0(rem0[0]));
	remainder_test test1(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd2), .done(remainder_done[1]), .rem0(rem0[1]));
	remainder_test test2(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd4), .done(remainder_done[2]), .rem0(rem0[2]));
	remainder_test test3(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd6), .done(remainder_done[3]), .rem0(rem0[3]));
	remainder_test test4(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd8), .done(remainder_done[4]), .rem0(rem0[4]));
	remainder_test test5(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd10), .done(remainder_done[5]), .rem0(rem0[5]));
	remainder_test test6(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd12), .done(remainder_done[6]), .rem0(rem0[6]));
	remainder_test test7(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd14), .done(remainder_done[7]), .rem0(rem0[7]));
	remainder_test test8(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd16), .done(remainder_done[8]), .rem0(rem0[8]));
	remainder_test test9(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd18), .done(remainder_done[9]), .rem0(rem0[9]));
	remainder_test test10(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd20), .done(remainder_done[10]), .rem0(rem0[10]));
	remainder_test test11(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd22), .done(remainder_done[11]), .rem0(rem0[11]));
	remainder_test test12(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd24), .done(remainder_done[12]), .rem0(rem0[12]));
	remainder_test test13(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd26), .done(remainder_done[13]), .rem0(rem0[13]));
	remainder_test test14(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd28), .done(remainder_done[14]), .rem0(rem0[14]));
	remainder_test test15(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd30), .done(remainder_done[15]), .rem0(rem0[15]));
	remainder_test test16(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd32), .done(remainder_done[16]), .rem0(rem0[16]));
	remainder_test test17(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd34), .done(remainder_done[17]), .rem0(rem0[17]));
	remainder_test test18(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd36), .done(remainder_done[18]), .rem0(rem0[18]));
	remainder_test test19(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd38), .done(remainder_done[19]), .rem0(rem0[19]));
	remainder_test test20(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd40), .done(remainder_done[20]), .rem0(rem0[20]));
	remainder_test test21(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd42), .done(remainder_done[21]), .rem0(rem0[21]));
	remainder_test test22(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd44), .done(remainder_done[22]), .rem0(rem0[22]));
	remainder_test test23(.clk(clk), .rst((rst | clear)), .numerator(dividend), .denominator(count[31:0] + 6'd46), .done(remainder_done[23]), .rem0(rem0[23]));
	
	
	reg [3:0] state;
	reg [3:0] state_c;
	
	
	//State Definitions
	parameter INITIAL = 2'b00;
	parameter PROCESS = 2'b01;
	parameter DONE = 2'b10;
	
	
	//Algorithm to test whether a number is prime
	always @(*) begin
	
		//Defaults
		count_c = count; //Number to divide by. Hold value
		is_prime_c = is_prime;
		done_c = done;
		state_c = state;
		remainder = remainder_c; //Fix output
		
		case (state)
			INITIAL: begin
				//reset at falling edge of start
				state_c = INITIAL;
				if (falling_transition == 1'b1) begin
					count_c = startValConcat; //Reset D to starting value
					done_c = 1'b0; //Not done with algorithm
					is_prime_c = 1'b1; //Defaults to prime unless proven otherwise
					//state_c = REM_RESET_HIGH;
					state_c = PROCESS;
					
					//Check if count is even. If so, make it odd by increasing it by one
					if (count_c[0] == 33'b0) begin
						count_c = count_c + 33'b1; 
					end
					
				end
				
				clear = 1'b1;
			end

			PROCESS: begin
			
				clear = 1'b0;
				
				//Check if 1 or 0
				if (dividend == 32'b0 | dividend == 32'b1) begin
					is_prime_c = 1'b0;
					done_c = 1'b1;
					state_c = INITIAL;
				end
				
				//Check if even number
				if (dividend[0] == 1'b0 & (dividend != 32'd2)) begin
					is_prime_c = 1'b0;
					done_c = 1'b1;
					state_c = INITIAL;
				end
				
				if (remainder_done == 24'b111111111111111111111111) begin //If remainder has finished running
			
					//Check whether the remainder is equal to 0
					
					if (rem0 != 24'b000000000000000000000000) begin
						is_prime_c = 1'b0; //Not a prime number if evenly divisible by anything
						done_c = 1'b1;
						state_c = INITIAL;
					end
					
					count_c = count + 33'd48; //Increment by 24 since there are 24 remainder testers
					
					//Reset remainder checker
					state_c = PROCESS;
					
					if ((count) > (endValConcat)) begin
						done_c = 1'b1;
						state_c = INITIAL;
					end
				end
				
				
			end
			DONE: begin
				
			end
		endcase
			
		//Reset
		if(rst == 1'b1) begin
			state_c = INITIAL;
			count_c = startValConcat;
			done_c = 1'b0; //Not done with algorithm
			is_prime_c = 1'b1; //Defaults to prime unless proven otherwise
		end
		
	end
	
	//FFs
	always @(posedge clk) begin
		count <= #1 count_c;
		is_prime <= #1 is_prime_c; 
		done <= #1 done_c;
		state <= #1 state_c;
	end
	
endmodule

//Finds the remainder of a number. Only needs to check odd % odd
module remainder_test(clk, rst, numerator, denominator, done, rem0);
	
	input clk, rst;
	input [31:0] numerator, denominator;
	reg [31:0] remainder;
	output reg done, rem0;
	reg done_c, rem0_c;
	
	reg [4:0] i; //Counter
	reg [4:0] i_c;
	reg [31:0] remainder_c;
	
	reg [31:0] Ni;
	
	always @(*) begin
		done_c = done;
		rem0_c = rem0;
		
		i_c = i - 5'b00001; //Subtract 1
		
		remainder_c = remainder << 1'b1;
		Ni = numerator << (5'b11111 - i);
		remainder_c[0] = Ni[31];
		if (remainder_c >= denominator) begin
			remainder_c = remainder_c - denominator;
		end
		
		//Check if odd numerator and even denominator
		if(numerator[0] == 1'b1 & denominator[0] == 1'b0) begin
			done_c = 1'b1;
			remainder_c = 32'b11; //Placeholder, anything will work as long as it's not 0
			rem0_c = 1'b0; //Placeholder, anything will work as long as its not 0
		end
		
		//If numerator < denominator, number is automatically not prime. If numerator == denominator, 
		//does not need to be tested. Definition of prime is that it can be divided by itself
		if (numerator <= denominator) begin
			done_c = 1'b1;
			remainder_c = 32'b11;
			rem0_c = 1'b0;
		end
		
		//Ignore Divide by 0 or 1
		if (denominator == 32'b0 | denominator == 32'b1) begin
			done_c = 1'b1;
			remainder_c = 32'b11; //Set remainder to number != 0
			rem0_c = 1'b0; 
		end
		
		//Return value
		if (i == 5'b00000) begin //Return value, reset
			done_c = 1'b1; //End
			
			if(remainder_c == 32'b0) begin //If the remainder is equal to zero
				rem0_c = 1'b1;
			end
			
		end
		else begin
			done_c = 1'b0;
		end
	
	
		if (rst == 1'b1 | done == 1'b1) begin //If reset or done. Done is used by the prime tester to reset the remainder tester
			i_c = 5'b11111;
			remainder_c = 32'b00000000000000000000000000000000;
			done_c = 1'b0;
			rem0_c = 1'b0;
		end
	end
	
	
	always @(posedge clk) begin
		i <= #1 i_c; //Counter
		remainder <= #1 remainder_c;
		done <= #1 done_c;
		rem0 <= #1 rem0_c;
	end
	
endmodule

//Generates a number to be tested. May be random or predetermined for debugging, depending on which debug switches are selected
module number_generator(clk, rst, sample, debug, value);
	input clk, rst, sample; //Sample connected to KEY0. Tells whether to generate a value
	input [1:0] debug; //Chose which number to display. (Random or predetermined for debugging)
	output reg [31:0] value;
	
	reg [15:0] count, count_c;
	reg [31:0] value_c;
	wire rising_transition, falling_transition;
	
	//Instantiate edge detector modules
	edge_detector_rising edge_detector_rising1(.clk(clk), .input_signal(sample), .rising_transition(rising_transition));
	edge_detector_falling edge_detector_falling1(.clk(clk), .input_signal(sample), .falling_transition(falling_transition));
		
	//Counter
	always @(*) begin
		//Default
		count_c = count + 16'b0000000000000001; //Add 1
		value_c = value;
		
		case (debug)
			2'b00: begin //Generate random number
				if (falling_transition == 1'b1) begin
					value_c = {value[31:16], count};
				end
				if(rising_transition == 1'b1) begin
					value_c = {count, value[15:0]};
				end
			end
			2'b01: begin //Constant 32-bit non prime number
				value_c = 32'b01111111111111111111111111111001;
			end
			2'b10: begin //Constant 32-bit prime number
				value_c = 32'd130680497;
			end
			2'b11: begin //Large 32-bit prime number 2^31 - 1
				value_c = 32'd2147483647;
			end
			
		endcase
		
		//Reset
		if (rst == 1'b1) begin
			count_c = 16'b0000000000000000; //Reset
			value_c = 32'b00000000000000000000000000000000;
		end
		
	end
	
	//FFs
	always @(*) begin
		count <= #1 count_c;
		value <= #1 value_c;
	end
	
endmodule

//Detects whether there is a rising signal.
module edge_detector_rising(clk, input_signal, rising_transition);
	
	input clk, input_signal;
	output reg rising_transition;
	
	reg n;
	wire rising_transition_c;
	
	assign rising_transition_c = ~n & input_signal;
	
	always @(posedge clk) begin
		n <= #1 input_signal;
		rising_transition <= rising_transition_c;
	end
	
endmodule

//Detects whether there is a falling signal.
module edge_detector_falling(clk, input_signal, falling_transition);
	
	input clk, input_signal;
	output reg falling_transition;
	
	reg n;
	wire falling_transition_c;
	
	assign falling_transition_c = n & ~input_signal;
	
	always @(posedge clk) begin
		n <= #1 input_signal;
		falling_transition <= falling_transition_c;
	end
	
endmodule
