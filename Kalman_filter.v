`timescale 1ns / 1ps

module kalman_filter_top_module(
    output reg signed [23:0] x_out,
    input signed [23:0] ang_vel, del_t, Q, R, z,
    input clk, rst
);
    reg signed [23:0] x_prev, P_prev;
    wire signed [23:0] x_pred, P_pred, k, new_x, new_P;
    
    //Predict step for mean and covariance 
    predict_mean UUT_pred (.x_pred(x_pred),.x_prev(x_prev),.ang_vel(ang_vel),.del_t(del_t));
    pred_covariance UUT_cov_pred (.P_pred(P_pred), .P_prev(P_prev), .Q(Q));
    gain_calc UUT_gain_c (.P_pred(P_pred), .R(R), .k(k));
    update_mean UUT_update (.new_x(new_x),.x_pred(x_pred),.k(k),.z(z));
    update_covariance UUT_upd_cov (.k(k), .P_pred(P_pred), .P_new(new_P));

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x_prev <= 24'sb0;
            P_prev <= 24'sb0;
            x_out  <= 24'sb0;
        end 
        else begin
            x_prev <= new_x;
            P_prev <= new_P;
            x_out  <= new_x;
        end
    end
endmodule

module predict_mean( //x_pred = x_prev + (ang_vel*del_t), predict step for mean
    output signed [23:0] x_pred,
    input signed [23:0] x_prev, ang_vel, del_t
);
/*
When performing (24 * 24) >>> 12 and storing the result in a 24 bit variable, the multiplication produces a 48 bit result. 
Since only 24 bits can be stored, the upper 24 bits are discarded.After that, the right shift is applied to the remaining 24 bits, 
which leads to incorrect values because the shift is no longer acting on the full 48?bit result.
Hence a 48 bit var is used to store the  entire multiplied resutl and then right shift by 12 is done
*/
    wire signed [47:0] product; // Use 48 bits to hold the result of 24x24 multiplication
    wire signed [23:0] control;

    assign product = ang_vel * del_t;
    assign control = product >>> 12; // Shift the 48-bit product before truncating to 24-bit
    assign x_pred  = x_prev + control;

endmodule 

module pred_covariance( //P_pred = P_prev + Q, predict the uncertainity 
    output signed [23:0] P_pred,
    input signed [23:0] P_prev,Q
);
    assign P_pred = P_prev + Q;
endmodule 

module gain_calc( //k = P_pred / (P_pred + R), kalman gain calculation, R: noise in the measurement z
    output signed [23:0] k,
    input signed [23:0] P_pred,R
);
/*
In numerator, when we do left shift by 12, its equivalent to multiplying P_pred by 2^12, 
If stored in a 24 bit wire, then the data is lost because of being truncated, hence we use a 48 bit wire
*/
    wire signed [23:0] denominator;
    wire signed [47:0] numerator; 
    assign numerator = P_pred <<< 12;
    assign denominator = P_pred + R;
    assign k = numerator / denominator;
endmodule
   
module update_mean( //x = x_pred + k(z-x_pred), update step for mean
    output signed [23:0] new_x,
    input signed [23:0] x_pred, k, z
);
/*
When performing (24 * 24) >>> 12 and storing the result in a 24 bit variable, the multiplication produces a 48 bit result. 
Since only 24 bits can be stored, the upper 24 bits are discarded.After that, the right shift is applied to the remaining 24 bits, 
which leads to incorrect values because the shift is no longer acting on the full 48?bit result.
Hence a 48 bit var is used to store the  entire multiplied resutl and then right shift by 12 is done
*/
    wire signed [23:0] error;
    wire signed [47:0] product;
    wire signed [23:0] correction;

    assign error = z - x_pred;
    assign product = k * error;
    assign correction = product >>> 12; // Shift the 48-bit product before truncating to 24-bit
    assign new_x = x_pred + correction;

endmodule

module update_covariance( //P_new = (1 - k)*P_pred, update the uncertainity after collecting the measurement value z
    output signed [23:0] P_new,
    input signed [23:0] P_pred,k
);
    wire signed [23:0] diff; 
    wire signed [47:0] product;
    
    assign diff = (4096 - k); //Q12,12 fixed point, hence 1*2^12=4096
    assign product = diff * P_pred;
    assign P_new = product >>> 12;
endmodule    

//TESTBENCH
module test_bench;
    reg signed [23:0] Q, R, z, dt, w;
    reg clk, rst;
    wire signed [23:0] x;
    reg signed [23:0] z_prev;
    
    reg signed [23:0] sensor_val [0:60];
    integer i; //used to sotre the time period of ramp signal  
    
    //To convert real values to fixed point integers 
    function signed [23:0] fixed_point_val;
        input real val;
        begin
            fixed_point_val = $rtoi(val * 4096.0);
        end
    endfunction

    kalman_filter_top_module DUT (.x_out(x),.ang_vel(w),.del_t(dt),.Q(Q),.R(R),.z(z),.clk(clk),.rst(rst));
  
    always #20 clk = ~clk; // Clock 40ns period

    initial begin
        clk = 0;
        rst = 1;
        
        //Store values into array
        $readmemh("const_mes.mem",sensor_val);
        z = sensor_val[0];
        z_prev=z;
        w = 0;
        i = 1;
        
        // initialize constants
        dt = fixed_point_val(0.1);
        Q = fixed_point_val(0.08);
        R = fixed_point_val(0.2);
    
        #10 rst = 0;
    end
    
    always @(posedge clk)
    begin 
        if(i==60) 
            $finish;
        if(!rst) begin //trigger only when rst is low                    
            z <= sensor_val[i]; //Measurement signal with noise 
            if(i!=0)
                w <= ((sensor_val[i]-z_prev)/dt); //find the angular velocity from the slope 
            z_prev <= sensor_val[i];
            i <= i+1;
            $strobe("Filter value=%.1f",x/4096.0);
        end
    end     
endmodule
