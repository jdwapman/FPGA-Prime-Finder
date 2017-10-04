//Testbench used to test the remainder_test,
//prime_tester, edge_detector_rising, edge_detector_falling, output_ram, serializer
//and number_generator modules

module fpga_prime_tb();
	
	//Inputs
	reg clk, rst;
	reg input_signal;
	reg sample;
	reg [1:0] debug;
	reg [31:0] numerator, denominator;
	
	//Outputs
	wire rising_transition, falling_transition;
	wire [31:0] value;
	wire done;
	wire [31:0] remainder;

	
	integer i, j;
	reg [31:0] inc;
	
	reg [31:0] dividend;
	reg start;
	wire [3:0] done_prime, is_prime;
	wire [32:0] count_0, count_1, count_2, count_3;
	wire prime_falling_transition;
	wire [31:0] remainder_prime;
	wire [23:0] remainder_prime_done;
	reg run;
	wire [23:0] rem00, rem01, rem02, rem03;
	
	edge_detector_rising edge_detector_rising1(.clk(clk), .input_signal(input_signal), .rising_transition(rising_transition));
	edge_detector_falling edge_detector_falling1(.clk(clk), .input_signal(input_signal), .falling_transition(falling_transition));
	number_generator gen1(.clk(clk), .rst(rst), .sample(sample), .debug(debug), .value(value));
	remainder_test remainder1 (.clk(clk), .rst(rst), .numerator(numerator), .denominator(denominator), .done(done), .rem0(rem0));
	
	//Prime Testers
	prime_tester prime0(.clk(clk), .rst(rst), .dividend(dividend), .start(start), .done(done_prime[0]), .is_prime(is_prime[0]), .count(count_0), .start_val(32'd2), .end_val((dividend >> 2)), .rem0(rem00)); //Divide by 16, check first half
	prime_tester prime1(.clk(clk), .rst(rst), .dividend(dividend), .start(start), .done(done_prime[1]), .is_prime(is_prime[1]), .count(count_1), .start_val((dividend >> 2)), .end_val((dividend >> 1)), .rem0(rem01)); //Divide by 16, check first half
	prime_tester prime2(.clk(clk), .rst(rst), .dividend(dividend), .start(start), .done(done_prime[2]), .is_prime(is_prime[2]), .count(count_2), .start_val((dividend >> 1)), .end_val((dividend >> 1) + (dividend >> 2)), .rem0(rem02)); //Divide by 16, check first half
	prime_tester prime3(.clk(clk), .rst(rst), .dividend(dividend), .start(start), .done(done_prime[3]), .is_prime(is_prime[3]), .count(count_3), .start_val((dividend >> 1) + (dividend >> 2)), .end_val((dividend >> 1) + 1'b1), .rem0(rem03)); //Divide by 16, check first half
	
	//Serializer
	reg save;
	reg [31:0] data_in;
	wire [7:0] write_data;
	wire write_enable;
	wire [1:0] write_addr;
	
	serializer ser1(.clk(clk), .rst(rst), .save(save), .data_in(data_in), .write_data(write_data), .write_enable(write_enable), .write_addr(write_addr));
	
	//Output RAM
	reg [1:0] read_addr;
	wire [7:0] read_data;
	output_ram ram1(.clk(clk), .write_data(write_data), .write_enable(write_enable), .write_addr(write_addr), .read_addr(read_addr), .read_data(read_data));
	
	initial begin
		
		#3;
		
		// Test rising edge detector
		$display("Test Edge Detector");
		input_signal = 1'b0;
		clk = 1'b0;
		
		#3;
		
		$display("clk: %b, input_signal: %b, rising_transition: %b, falling_transition: %b", clk, input_signal, rising_transition, falling_transition);
		
		repeat(5) begin
			clk = ~clk;
			#3;
			$display("clk: %b, input_signal: %b, rising_transition: %b, falling_transition: %b", clk, input_signal, rising_transition, falling_transition);
		end
		
		input_signal = 1'b1;
		
		repeat(5) begin
			clk = ~clk;
			#3;
			$display("clk: %b, input_signal: %b, rising_transition: %b, falling_transition: %b", clk, input_signal, rising_transition, falling_transition);
		end
		
		input_signal = 1'b0;
		
		repeat(5) begin
			clk = ~clk;
			#3;
			$display("clk: %b, input_signal: %b, rising_transition: %b, falling_transition: %b", clk, input_signal, rising_transition, falling_transition);
		end
		
		//Test random number generator
		$display("Random Number Generator");
		#1;
		debug = 2'b00;
		clk = 1'b0;
		rst = 1'b1; //Reset
		run = 1'b1;
		#3;
		
		rst = 1'b0;
		
		repeat(20) begin
			#1;
			clk = ~clk;
		end
		
		debug = 2'b01;
		repeat(20) begin
			#1;
			clk = ~clk;
		end
		#1;
		$display("Debug: %b, value: %d", debug, value);
		
		debug = 2'b10;
		repeat(20) begin
			#1;
			clk = ~clk;
		end
		#1;
		$display("Debug: %b, value: %d", debug, value);
		
		debug = 2'b11;
		repeat(20) begin
			#1;
			clk = ~clk;
		end
		#1;
		$display("Debug: %b, value: %d", debug, value);
		
		debug = 2'b00; //Generate random number
		sample = 1'b1;
		//Change repeat count to change random number
		repeat(2000) begin //Wait
			#1;
			clk = ~clk;
		end
		
		sample = 1'b0; //Go low
		repeat(4000) begin //Wait
			#1;
			clk = ~clk;
		end
		
		sample = 1'b1; //Go high
		repeat(6000) begin //Wait
			#1;
			clk = ~clk;
		end
		
		
		#1;
		$display("Debug: %b, value: %d", debug, value);
		
		//Remainder test
		$display("Remainder Test");
		numerator = 5;
		denominator = 6;
		clk = 1'b0;
		rst = 1'b0;
		#1;
		rst = 1'b1;
		

		
		repeat(2) begin
			#1;
			clk = ~clk;
			$display("Num: %d, Den: %d, Rem: %d, Done: %d", numerator, denominator, remainder, done);
		end
		rst = 1'b0;
		#1;
		
		for (i = 1; i < 10; i = i + 1) begin
			for (j = 1; j < 10; j = j + 1) begin
				/*
				
				#1;
				rst = 1'b1;
				#1;
				repeat(20) begin
					#1;
					clk = ~clk;
				end
				rst = 1'b0;
				#1;
				
				*/
				numerator = i;
				denominator = j;
				#1;
			
				while (done == 1'b0) begin
					#1;
					clk = ~clk;
				end
				#1;
				$display("Num: %d, Den: %d, Done: %d, Rem0: %d", numerator, denominator, done, rem0);
			end
		end
		
		
		//Test prime number tester
		$display("Test Prime Number");
		
			clk = 1'b0;
			rst = 1'b0;
			#3;
			clk = 1'b1;
			rst = 1'b1;
			#3;
			rst = 1'b0; //Bring reset low
		
		inc = 32'd2;
		repeat (100) begin
			
			clk = 1'b0;
			rst = 1'b0;
			#3;
			clk = 1'b1;
			rst = 1'b1;
			#3;
			rst = 1'b0; //Bring reset low
			#3;
			
			start = 1'b1;
			repeat(3) begin //Wait
				#3;
				clk = ~clk;
			end
			#3;
			start = 1'b0;
			#3;
			dividend = inc;
			#3;
			while (done_prime != 4'b1111) begin //Wait
				#3;
				clk = ~clk;
				
				//$display("inc: %d, isPrime: %b, done: %b, count_0: %d , count_1: %d, count_2: %d, count_3: %d", inc, is_prime, done_prime, count_0, count_1, count_2, count_3); //State 0
			end
			#3;
			$display("inc: %d, isPrime: %b, done: %b", inc, &is_prime, done_prime); //State 0
			#3;
			inc = inc + 64'b1;
		end
		
		$display("Test Serializer, Output RAM");
		//Initialize
		clk = 1'b0;
		save = 1'b0;
		rst = 1'b0;
		data_in = 32'b11111111111111101111110011111000;
		repeat(6) begin //Wait
			#3;
			clk = ~clk;
		end
		
		rst = 1'b1; //Reset
		
		repeat(6) begin //Wait
				#3;
				clk = ~clk;
		end
		
		rst = 1'b0;
		
		repeat(6) begin //Wait
			#3;
			clk = ~clk;
		end
		
		save = 1'b1;
		repeat(20) begin
			#3;
			clk = ~clk;
			if (clk == 1'b1) begin
				$display("write_data: %b, write_enable: %b, write_addr: %b", write_data, write_enable, write_addr);
				
			end
			
		end
		
		$display("Test Output RAM");
		read_addr = 2'b00;
		repeat(4) begin //Wait
			#3;
			clk = ~clk;
			#3;
			$display("read_addr: %b, read_data: %b", read_addr, read_data);
			read_addr = read_addr + 1'b1;
		end
		
	end
	
endmodule
