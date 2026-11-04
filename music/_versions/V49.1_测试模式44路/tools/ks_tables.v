    // ===== 由 tools\ks_model.py 自动生成：36 键弦参数 =====
    //   NI_TAB : 延迟线回绕长度（环路延迟 = NI + FR/64 + 0.5 个采样）
    //   FR_TAB : 线性插值的小数系数（1/64 采样），用来把音调修准
    //   DMP_TAB: 每绕一圈的损耗位移（gain = 1 - 2^-DMP），定衰减时间
    function [9:0] NI_TAB(input [5:0] k);
        case (k)
            6'd0 : NI_TAB = 10'd835 ;  6'd1 : NI_TAB = 10'd745 ;  6'd2 : NI_TAB = 10'd626 ;  6'd3 : NI_TAB = 10'd558 ;  
            6'd4 : NI_TAB = 10'd498 ;  6'd5 : NI_TAB = 10'd888 ;  6'd6 : NI_TAB = 10'd794 ;  6'd7 : NI_TAB = 10'd703 ;  
            6'd8 : NI_TAB = 10'd664 ;  6'd9 : NI_TAB = 10'd592 ;  6'd10: NI_TAB = 10'd528 ;  6'd11: NI_TAB = 10'd469 ;  
            6'd12: NI_TAB = 10'd419 ;  6'd13: NI_TAB = 10'd373 ;  6'd14: NI_TAB = 10'd314 ;  6'd15: NI_TAB = 10'd280 ;  
            6'd16: NI_TAB = 10'd249 ;  6'd17: NI_TAB = 10'd444 ;  6'd18: NI_TAB = 10'd395 ;  6'd19: NI_TAB = 10'd353 ;  
            6'd20: NI_TAB = 10'd332 ;  6'd21: NI_TAB = 10'd296 ;  6'd22: NI_TAB = 10'd264 ;  6'd23: NI_TAB = 10'd235 ;  
            6'd24: NI_TAB = 10'd210 ;  6'd25: NI_TAB = 10'd187 ;  6'd26: NI_TAB = 10'd157 ;  6'd27: NI_TAB = 10'd140 ;  
            6'd28: NI_TAB = 10'd125 ;  6'd29: NI_TAB = 10'd222 ;  6'd30: NI_TAB = 10'd198 ;  6'd31: NI_TAB = 10'd176 ;  
            6'd32: NI_TAB = 10'd166 ;  6'd33: NI_TAB = 10'd148 ;  6'd34: NI_TAB = 10'd132 ;  6'd35: NI_TAB = 10'd118 ;  
            default: NI_TAB = 10'd444;
        endcase
    endfunction

    function [5:0] FR_TAB(input [5:0] k);
        case (k)
            6'd0 : FR_TAB = 6'd53;  6'd1 : FR_TAB = 6'd2 ;  6'd2 : FR_TAB = 6'd32;  6'd3 : FR_TAB = 6'd30;  6'd4 : FR_TAB = 6'd16;  6'd5 : FR_TAB = 6'd46;  
            6'd6 : FR_TAB = 6'd35;  6'd7 : FR_TAB = 6'd60;  6'd8 : FR_TAB = 6'd11;  6'd9 : FR_TAB = 6'd41;  6'd10: FR_TAB = 6'd40;  6'd11: FR_TAB = 6'd0 ;  
            6'd12: FR_TAB = 6'd24;  6'd13: FR_TAB = 6'd49;  6'd14: FR_TAB = 6'd32;  6'd15: FR_TAB = 6'd44;  6'd16: FR_TAB = 6'd24;  6'd17: FR_TAB = 6'd39;  
            6'd18: FR_TAB = 6'd8 ;  6'd19: FR_TAB = 6'd61;  6'd20: FR_TAB = 6'd21;  6'd21: FR_TAB = 6'd37;  6'd22: FR_TAB = 6'd36;  6'd23: FR_TAB = 6'd12;  
            6'd24: FR_TAB = 6'd60;  6'd25: FR_TAB = 6'd50;  6'd26: FR_TAB = 6'd32;  6'd27: FR_TAB = 6'd38;  6'd28: FR_TAB = 6'd60;  6'd29: FR_TAB = 6'd35;  
            6'd30: FR_TAB = 6'd52;  6'd31: FR_TAB = 6'd14;  6'd32: FR_TAB = 6'd9 ;  6'd33: FR_TAB = 6'd20;  6'd34: FR_TAB = 6'd34;  6'd35: FR_TAB = 6'd63;  
            default: FR_TAB = 6'd0;
        endcase
    endfunction

    function [3:0] DMP_TAB(input [5:0] k);
        case (k)
            6'd0 : DMP_TAB = 4'd6 ;  6'd1 : DMP_TAB = 4'd6 ;  6'd2 : DMP_TAB = 4'd6 ;  6'd3 : DMP_TAB = 4'd6 ;  6'd4 : DMP_TAB = 4'd6 ;  6'd5 : DMP_TAB = 4'd6 ;  6'd6 : DMP_TAB = 4'd6 ;  6'd7 : DMP_TAB = 4'd6 ;  6'd8 : DMP_TAB = 4'd6 ;  
            6'd9 : DMP_TAB = 4'd6 ;  6'd10: DMP_TAB = 4'd6 ;  6'd11: DMP_TAB = 4'd6 ;  6'd12: DMP_TAB = 4'd6 ;  6'd13: DMP_TAB = 4'd6 ;  6'd14: DMP_TAB = 4'd6 ;  6'd15: DMP_TAB = 4'd6 ;  6'd16: DMP_TAB = 4'd6 ;  6'd17: DMP_TAB = 4'd6 ;  
            6'd18: DMP_TAB = 4'd6 ;  6'd19: DMP_TAB = 4'd6 ;  6'd20: DMP_TAB = 4'd6 ;  6'd21: DMP_TAB = 4'd6 ;  6'd22: DMP_TAB = 4'd6 ;  6'd23: DMP_TAB = 4'd7 ;  6'd24: DMP_TAB = 4'd7 ;  6'd25: DMP_TAB = 4'd7 ;  6'd26: DMP_TAB = 4'd7 ;  
            6'd27: DMP_TAB = 4'd7 ;  6'd28: DMP_TAB = 4'd7 ;  6'd29: DMP_TAB = 4'd7 ;  6'd30: DMP_TAB = 4'd7 ;  6'd31: DMP_TAB = 4'd7 ;  6'd32: DMP_TAB = 4'd7 ;  6'd33: DMP_TAB = 4'd7 ;  6'd34: DMP_TAB = 4'd7 ;  6'd35: DMP_TAB = 4'd7 ;  
            default: DMP_TAB = 4'd6;
        endcase
    endfunction
