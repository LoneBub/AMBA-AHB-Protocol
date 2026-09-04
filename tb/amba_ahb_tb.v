//test-bench
module tb_my_ahb_project_final;
reg hclk_a,hrst_a,hbusreq_m4_a;

my_ahb_project_final ahb_final(hclk_a,hrst_a,hbusreq_m4_a);

always #5 hclk_a = ~hclk_a;
initial begin
hrst_a = 0;
hclk_a = 0;
#3 hrst_a = 1;
//hbusreq_m3_a = 0;
hbusreq_m4_a = 0;
end
endmodule