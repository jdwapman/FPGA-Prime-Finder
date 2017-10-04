//Testbench used to test the next prime finder of Lab 7

module lab7_tb2();

	reg ADC_CLK_10; //Input
	wire MAX10_CLK1_50, MAX10_CLK2_50; //Input
	wire [7:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
	reg [1:0] KEY; //Input
	wire [9:0] LEDR;
	reg [9:0] SW; //Input
	wire [31:0] count, orig_val, prime_val;
	reg [31:0] test_number; //Input
	wire [2:0] state;
	wire LED_done, LED_prime;
	reg clk, rst;
	wire [7:0] done;
	wire [7:0] is_prime;
	integer i;
	
	lab7_multi UUT(
					.ADC_CLK_10(ADC_CLK_10),
					.MAX10_CLK1_50(MAX10_CLK1_50),
					.MAX10_CLK2_50(MAX10_CLK2_50),
					.HEX0(HEX0),
					.HEX1(HEX1),
					.HEX2(HEX2),
					.HEX3(HEX3),
					.HEX4(HEX4),
					.HEX5(HEX5),
					.KEY(KEY),
					.LEDR(LEDR),
					.SW(SW),
					.count(count),
					.orig_val(orig_val),
					.prime_val(prime_val),
					.test_number(test_number),
					.state(state),
					.LED_done(LED_done),
					.LED_prime(LED_prime),
					.done(done),
					.is_prime(is_prime)
					);
					
	assign MAX10_CLK1_50 = clk;
	
					
	initial begin
	
		//Initialize variables
		test_number = 32'd1000; //Number to test
		clk = 1'b0;
		SW[9:0] = 10'b0000000000;
		KEY[1:0] = 2'b11; //Keys are active low
		
		//Reset
		repeat (100) begin
			#3;
			clk = ~clk;
			#3;
		end
		
		SW[9] = 1'b1; //Reset
		
		repeat (100) begin
			#3;
			clk = ~clk;
			#3;
		end
		
		SW[9] = 1'b0; //Reset
		//repeat(5) begin
		
			repeat (100) begin
				#3;
				clk = ~clk;
				#3;
				//$display("state: %b, testing: %d, done: %b, orig: %d, is_prime: %b, prime_val: %d, LED_done: %b, LED_prime: %b", state, count, done, orig_val, is_prime, prime_val, LED_done, LED_prime);
			end
			
			KEY[1] = 1'b1;
			
			repeat (100) begin
				#3;
				clk = ~clk;
				#3;
			end
			
			KEY[1] = 1'b0;
			
			while(LED_done != 1'b1) begin
				#3;
				clk = ~clk;
				#3;
				if (clk == 1'b1) begin
					$display("test_number: %d, state: %b, testing: %d, done: %b, orig: %d, is_prime: %b, prime_val: %d, LED_done: %b, LED_prime: %b", test_number, state, count, done, orig_val, is_prime, prime_val, LED_done, LED_prime);
				end
			end
			/*
			repeat (30) begin
				#3;
				clk = ~clk;
				#3;
			end
			*/
			$display("test_number: %d, state: %b, testing: %d, done: %b, orig: %d, is_prime: %b, prime_val: %d, LED_done: %b, LED_prime: %b", test_number, state, count, done, orig_val, is_prime, prime_val, LED_done, LED_prime);
			
			
			repeat (100) begin
				#3;
				clk = ~clk;
				#3;
				//$display("state: %b, testing: %d, done: %b, orig: %d, is_prime: %b, prime_val: %d, LED_done: %b, LED_prime: %b", state, count, done, orig_val, is_prime, prime_val, LED_done, LED_prime);
			end
			
			$display("test_number: %d, state: %b, testing: %d, done: %b, orig: %d, is_prime: %b, prime_val: %d, LED_done: %b, LED_prime: %b", test_number, state, count, done, orig_val, is_prime, prime_val, LED_done, LED_prime);
			
			KEY[1] = 1'b1;
			
			repeat (100) begin
				#3;
				clk = ~clk;
				#3;
			end
			test_number = 32'd1164;
			KEY[1] = 1'b0;
			#3;
			
			repeat (10) begin
				#3;
				clk = ~clk;
				#3;
			end
			$display("Here");
			$display("test_number: %d, state: %b, testing: %d, done: %b, orig: %d, is_prime: %b, prime_val: %d, LED_done: %b, LED_prime: %b", test_number, state, count, done, orig_val, is_prime, prime_val, LED_done, LED_prime);

			while (LED_done != 1'b1) begin
				#3;
				clk = ~clk;
				#3;
				if(clk == 1'b1) begin
					$display("test_number: %d, state: %b, testing: %d, done: %b, orig: %d, is_prime: %b, prime_val: %d, LED_done: %b, LED_prime: %b", test_number, state, count, done, orig_val, is_prime, prime_val, LED_done, LED_prime);
				end
			end
			$display("Here2");
			
		//end
	end

endmodule