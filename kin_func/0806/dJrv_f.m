function dJrv = dJrv_f(z6,z7,z12,z13,z14,z15,z16,z17,z18,z19)
%DJRV_F
%    DJRV = DJRV_F(Z6,Z7,Z12,Z13,Z14,Z15,Z16,Z17,Z18,Z19)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    05-Aug-2019 09:19:24

t2 = conj(z6);
t3 = conj(z7);
t4 = conj(z12);
t5 = conj(z13);
t6 = conj(z14);
t7 = conj(z15);
t8 = conj(z16);
t9 = conj(z17);
t10 = conj(z18);
t11 = conj(z19);
t12 = cos(t2);
t13 = cos(t4);
t14 = cos(t6);
t15 = cos(t8);
t16 = cos(t10);
t17 = sin(t2);
t18 = sin(t4);
t19 = sin(t6);
t20 = sin(t8);
t21 = sin(t10);
t22 = t17.*t18;
t23 = t19.*t20;
t24 = t12.*t13;
t25 = t14.*t15;
t26 = t12.*t18;
t27 = t13.*t17;
t28 = t14.*t20;
t29 = t15.*t19;
t30 = -t24;
t31 = -t25;
t32 = t28.*(1.3e+1./1.0e+2);
t33 = t29.*(1.3e+1./1.0e+2);
t34 = t26+t27;
t35 = t28+t29;
t36 = t22+t30;
t37 = t23+t31;
t38 = t14.*t34;
t39 = t19.*t34;
t43 = t16.*t35.*(3.0./2.5e+1);
t40 = t14.*t36;
t41 = t19.*t36;
t42 = t38.*(3.0./2.5e+1);
t45 = t39.*(3.0./2.5e+1);
t47 = t38.*(3.0./1.0e+2);
t48 = t39.*(3.0./1.0e+2);
t51 = t21.*t37.*(3.0./2.5e+1);
t58 = t38.*6.123233995736766e-17;
t59 = t39.*6.123233995736766e-17;
t62 = t38.*7.347880794884119e-18;
t63 = t39.*7.347880794884119e-18;
t66 = t38.*1.83697019872103e-18;
t67 = t39.*1.83697019872103e-18;
t44 = -t41;
t46 = -t42;
t49 = t40.*(3.0./2.5e+1);
t50 = t41.*(3.0./2.5e+1);
t52 = t40.*(3.0./1.0e+2);
t53 = -t51;
t54 = t41.*(3.0./1.0e+2);
t60 = t40.*6.123233995736766e-17;
t61 = t41.*6.123233995736766e-17;
t65 = -t62;
t68 = t40.*7.347880794884119e-18;
t69 = t41.*7.347880794884119e-18;
t70 = t40.*1.83697019872103e-18;
t71 = t41.*1.83697019872103e-18;
t75 = t40+t59;
t55 = -t54;
t56 = t43+t53;
t64 = -t61;
t72 = -t71;
t76 = t39+t60;
t78 = t44+t58;
t79 = t15.*t75;
t81 = t20.*t75;
t57 = t11.*t56;
t73 = t32+t33+t56;
t77 = t38+t64;
t80 = t15.*t76;
t82 = t20.*t76;
t85 = t15.*t78;
t86 = t20.*t78;
t87 = -t81;
t89 = t79.*(1.3e+1./1.0e+2);
t91 = t81.*(1.3e+1./1.0e+2);
t74 = t9.*t73;
t83 = t15.*t77;
t84 = t20.*t77;
t88 = -t82;
t90 = t80.*(1.3e+1./1.0e+2);
t92 = t82.*(1.3e+1./1.0e+2);
t95 = -t91;
t97 = t85.*(1.3e+1./1.0e+2);
t98 = t86.*(1.3e+1./1.0e+2);
t100 = t79+t86;
t102 = t85+t87;
t111 = t16.*(t81-t85).*(-3.0./2.5e+1);
t112 = t21.*(t81-t85).*(-3.0./2.5e+1);
t93 = t83.*(1.3e+1./1.0e+2);
t94 = t84.*(1.3e+1./1.0e+2);
t96 = -t92;
t99 = t80+t84;
t101 = t83+t88;
t105 = t16.*t100.*(3.0./2.5e+1);
t107 = t21.*t100.*(3.0./2.5e+1);
t108 = t16.*(t82-t83).*(-3.0./2.5e+1);
t109 = t21.*(t82-t83).*(-3.0./2.5e+1);
t103 = t16.*t99.*(3.0./2.5e+1);
t104 = t21.*t99.*(3.0./2.5e+1);
t110 = -t107;
t115 = t105+t112;
t121 = -t11.*(t107+t16.*(t81-t85).*(3.0./2.5e+1));
t122 = t11.*(t107+t16.*(t81-t85).*(3.0./2.5e+1));
t131 = -t9.*(t91-t97+t107+t16.*(t81-t85).*(3.0./2.5e+1));
t132 = t9.*(t91-t97+t107+t16.*(t81-t85).*(3.0./2.5e+1));
t106 = -t104;
t113 = t103+t109;
t116 = t110+t111;
t118 = -t11.*(t104+t16.*(t82-t83).*(3.0./2.5e+1));
t119 = t11.*t115;
t120 = t11.*(t104+t16.*(t82-t83).*(3.0./2.5e+1));
t126 = t89+t98+t115;
t127 = -t9.*(t92-t93+t104+t16.*(t82-t83).*(3.0./2.5e+1));
t130 = t9.*(t92-t93+t104+t16.*(t82-t83).*(3.0./2.5e+1));
t114 = t106+t108;
t117 = t11.*t113;
t123 = t90+t94+t113;
t128 = t95+t97+t116;
t129 = t9.*t126;
t135 = t50+t52+t65+t67+t126;
t124 = t93+t96+t114;
t125 = t9.*t123;
t133 = t46+t48+t69+t70+t123;
t137 = t49+t55+t63+t66+t128;
t139 = t5.*t135;
t134 = t45+t47+t68+t72+t124;
t136 = t7.*t133;
t141 = t7.*t137;
t138 = t5.*t134;
t142 = -t141;
t140 = -t138;
dJrv = reshape([-t3.*t17,t3.*t12,0.0,t117+t125+t136+t139+t3.*(t12.*(3.0./2.0e+1)+t135),t122+t132+t140+t142+t3.*(t17.*(3.0./2.0e+1)-t134),0.0,t117+t125+t136+t139+t3.*t135,t122+t132+t140+t142-t3.*t134,0.0,t119+t129+t3.*t133+t5.*t133+t7.*t135,t120+t130-t3.*t137-t7.*t134-t5.*t137,t57+t74+t7.*(t14.*(-3.0./2.5e+1)+t19.*(3.0./1.0e+2)+t73),t119+t129+t3.*t123+t5.*t123+t7.*t126,t120+t130+t7.*(t92-t93+t104+t16.*(t82-t83).*(3.0./2.5e+1))+t3.*(t91-t97+t107+t16.*(t81-t85).*(3.0./2.5e+1))+t5.*(t91-t97+t107+t16.*(t81-t85).*(3.0./2.5e+1)),t57+t74+t7.*t73,t119+t3.*t113+t5.*t113+t7.*t115+t9.*t115,t120+t3.*(t107+t16.*(t81-t85).*(3.0./2.5e+1))+t7.*(t104+t16.*(t82-t83).*(3.0./2.5e+1))+t5.*(t107+t16.*(t81-t85).*(3.0./2.5e+1))+t9.*(t104+t16.*(t82-t83).*(3.0./2.5e+1)),t57+t7.*t56+t9.*t56],[3,6]);
