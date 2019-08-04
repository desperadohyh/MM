function J_end = Jk_f(z9,z11,z12,z13,z14,z15,z16,z17,z18,zk6,zk9,zk11)
%JK_F
%    J_END = JK_F(Z9,Z11,Z12,Z13,Z14,Z15,Z16,Z17,Z18,ZK6,ZK9,ZK11)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    03-Aug-2019 17:13:58

t2 = conj(z9);
t3 = conj(z11);
t4 = conj(z13);
t5 = conj(z15);
t6 = conj(z17);
t7 = cos(z12);
t8 = cos(z14);
t9 = cos(z16);
t10 = cos(z18);
t11 = conj(zk6);
t12 = conj(zk9);
t13 = conj(zk11);
t14 = sin(z12);
t15 = sin(z14);
t16 = sin(z16);
t17 = sin(z18);
t33 = z9./5.0;
t34 = z11./5.0;
t36 = z9./5.0e+1;
t37 = z11./5.0e+1;
t43 = z9.*(3.0./1.0e+2);
t44 = z11.*(3.0./1.0e+2);
t155 = z9.*3.67394039744206e-19;
t156 = z11.*3.67394039744206e-19;
t158 = z9.*1.469576158976824e-18;
t159 = z9.*1.592040838891559e-18;
t161 = z11.*1.469576158976824e-18;
t162 = z11.*1.592040838891559e-18;
t247 = z9.*3.67394039744206e-19;
t248 = z11.*3.67394039744206e-19;
t18 = cos(t11);
t19 = sin(t11);
t20 = t7.^2;
t21 = t8.^2;
t22 = t9.^2;
t23 = t10.^2;
t24 = t14.^2;
t25 = t15.^2;
t26 = t16.^2;
t27 = t17.^2;
t28 = t8.*t9;
t29 = t12+t13;
t30 = t8.*t16;
t31 = t9.*t15;
t32 = t15.*t16;
t35 = -t34;
t39 = t5.*(3.0./2.5e+1);
t40 = t2./5.0e+1;
t41 = t3./5.0e+1;
t42 = -t37;
t46 = t5.*(3.0./1.0e+2);
t47 = -t44;
t48 = t5.*(1.3e+1./1.0e+2);
t49 = t6.*(1.3e+1./1.0e+2);
t55 = t2.*8.5e-3;
t56 = t3.*8.5e-3;
t160 = -t156;
t164 = t4.*1.83697019872103e-18;
t165 = -t162;
t166 = -t161;
t167 = t4.*7.347880794884119e-18;
t168 = t4.*7.960204194457796e-18;
t174 = t5.*3.0e-2;
t249 = t4.*1.83697019872103e-18;
t250 = -t248;
t38 = -t32;
t45 = -t41;
t50 = t18.*(3.0./2.0e+2);
t51 = t20+t24;
t52 = t21+t25;
t53 = t22+t26;
t54 = t23+t27;
t58 = -t56;
t61 = t30+t31;
t66 = t2.*t18.*(-3.0./2.0e+2);
t67 = t3.*t18.*(-3.0./2.0e+2);
t70 = t4+t33+t35;
t72 = t36+t42+zk6;
t75 = t19.*t29.*3.0e-4;
t57 = -t50;
t59 = t2.*t50;
t60 = t3.*t50;
t62 = 1.0./t51;
t63 = 1.0./t52;
t64 = 1.0./t53;
t65 = 1.0./t54;
t68 = t28+t38;
t69 = t17.*t61;
t71 = t10.*t61;
t73 = sin(t72);
t76 = cos(t72);
t78 = -t75;
t81 = t11+t40+t45;
t93 = t55+t58;
t74 = t17.*t68;
t77 = t10.*t68;
t79 = t76.^2;
t80 = t73.^2;
t83 = cos(t81);
t84 = sin(t81);
t85 = t7.*t76;
t86 = t7.*t73;
t87 = t14.*t76;
t88 = t14.*t73;
t89 = t8.*t63.*(3.0./2.5e+1);
t90 = t15.*t63.*(3.0./2.5e+1);
t102 = t19.*t29.*t81.*(3.0./2.0e+2);
t169 = t15.*t63.*1.83697019872103e-18;
t170 = t8.*t63.*1.83697019872103e-18;
t171 = t15.*t63.*7.347880794884119e-18;
t172 = t8.*t63.*7.347880794884119e-18;
t189 = t28.*t63.*t64.*1.592040838891559e-18;
t190 = t32.*t63.*t64.*1.592040838891559e-18;
t191 = t8.*t63.*6.0e-3;
t192 = t8.*t63.*3.0e-2;
t193 = t15.*t63.*1.2e-1;
t194 = t15.*t63.*2.4e-2;
t211 = t30.*t63.*t64.*1.0;
t212 = t31.*t63.*t64.*1.0;
t213 = t32.*t63.*t64.*1.0;
t214 = t28.*t63.*t64.*2.0e-1;
t215 = t30.*t63.*t64.*2.0e-1;
t216 = t31.*t63.*t64.*2.0e-1;
t217 = t32.*t63.*t64.*2.0e-1;
t219 = t28.*t63.*t64.*1.0;
t221 = t28.*t63.*t64.*1.3e-1;
t222 = t32.*t63.*t64.*1.3e-1;
t224 = t28.*t63.*t64.*2.6e-2;
t226 = t32.*t63.*t64.*2.6e-2;
t236 = t32.*t63.*t64.*t70.*(-1.3e-1);
t242 = t8.*t63.*2.249639673992787e-35;
t243 = t15.*t63.*8.998558695971146e-35;
t244 = t8.*t63.*1.124819836996393e-34;
t245 = t15.*t63.*1.124819836996393e-34;
t251 = t8.*t63.*3.67394039744206e-19;
t252 = t15.*t63.*1.469576158976824e-18;
t253 = t8.*t63.*1.83697019872103e-18;
t254 = t15.*t63.*7.347880794884119e-18;
t82 = -t77;
t91 = -t90;
t92 = -t88;
t94 = t85./5.0e+1;
t95 = t86./5.0e+1;
t96 = t87./5.0e+1;
t97 = t88./5.0e+1;
t99 = t83.*8.5e-3;
t100 = t84.*8.5e-3;
t101 = t71+t74;
t104 = -t102;
t105 = t79+t80;
t106 = t86+t87;
t111 = t83.*t93;
t112 = t84.*t93;
t173 = -t171;
t195 = t89+t169;
t218 = t70.*t192;
t220 = -t213;
t223 = -t217;
t225 = t70.*t193;
t227 = -t222;
t228 = -t226;
t234 = t70.*t221;
t235 = t70.*t222;
t255 = t70.*t253;
t256 = t70.*t254;
t282 = t192+t193;
t287 = t191+t194;
t307 = t211+t212;
t308 = t215+t216;
t316 = -t17.*t65.*(t213-t219);
t319 = -t10.*t65.*(t213-t219);
t322 = t10.*t65.*(t213-t219);
t98 = -t97;
t103 = t69+t82;
t107 = 1.0./t105;
t108 = t85+t92;
t109 = t8.*t106;
t110 = t15.*t106;
t116 = t111./5.0e+1;
t117 = t112./5.0e+1;
t119 = t95+t96;
t131 = t59+t60+t104+t112;
t132 = t66+t67+t102+t111;
t144 = t10.*t65.*(t102+t111+t2.*t57+t3.*t57);
t145 = t17.*t65.*(t102+t111+t2.*t57+t3.*t57);
t210 = t91+t170+1.3e+1./1.0e+2;
t229 = t9.*t64.*t195;
t230 = t16.*t64.*t195;
t309 = t218+t225;
t310 = t219+t220;
t311 = t214+t223;
t312 = t17.*t65.*t307;
t313 = t10.*t65.*t308;
t314 = t17.*t65.*t308;
t315 = t10.*t65.*t307;
t324 = t8.*t63.*t282.*1.0;
t325 = t15.*t63.*t282.*1.0;
t326 = t8.*t63.*t287.*1.0;
t327 = t15.*t63.*t287.*1.0;
t370 = t221+t227+t253+t254+1.83697019872103e-18;
t113 = t8.*t108;
t114 = t15.*t108;
t115 = -t110;
t118 = -t117;
t120 = t94+t98;
t121 = t8.*t119;
t122 = t15.*t119;
t126 = t109.*6.123233995736766e-17;
t127 = t110.*6.123233995736766e-17;
t136 = t50+t78+t100+t116;
t141 = t57+t78+t100+t116;
t142 = t10.*t65.*t131;
t143 = t17.*t65.*t131;
t150 = -t10.*t65.*(t50+t78-t99+t117);
t151 = -t10.*t65.*(t50+t75-t100-t116);
t152 = -t17.*t65.*(t50+t78-t99+t117);
t153 = -t17.*t65.*(t50+t75-t100-t116);
t154 = -t145;
t163 = t17.*t65.*(t50+t78-t99+t117);
t231 = -t230;
t232 = t9.*t64.*t210;
t233 = t16.*t64.*t210;
t317 = t10.*t65.*t311;
t318 = t17.*t65.*t311;
t321 = -t314;
t328 = -t324;
t329 = -t325;
t330 = -t326;
t331 = t8.*t63.*t309.*1.0;
t332 = t15.*t63.*t309.*1.0;
t353 = t315+t316;
t355 = t312+t322;
t367 = -t9.*t64.*(t171-t244+t324-7.960204194457796e-18);
t368 = -t16.*t64.*(t171-t244+t324-7.960204194457796e-18);
t369 = t9.*t64.*(t171-t244+t324-7.960204194457796e-18);
t123 = t8.*t120;
t124 = t15.*t120;
t125 = -t122;
t128 = -t127;
t129 = t113.*6.123233995736766e-17;
t130 = t114.*6.123233995736766e-17;
t133 = t121.*6.123233995736766e-17;
t134 = t122.*6.123233995736766e-17;
t135 = t50+t75+t99+t118;
t140 = t57+t75+t99+t118;
t147 = t10.*t65.*t136;
t149 = t17.*t65.*t136;
t176 = t114+t126;
t237 = t142+t154;
t246 = t43+t47+t143+t144;
t320 = t229+t233;
t323 = t231+t232;
t333 = -t331;
t334 = -t332;
t352 = t313+t318;
t354 = t317+t321;
t358 = -t10.*(t314-t317);
t359 = -t17.*(t314-t317);
t363 = t172+t245+t329;
t366 = t173+t244+t328+7.960204194457796e-18;
t137 = t123.*6.123233995736766e-17;
t138 = t124.*6.123233995736766e-17;
t139 = -t134;
t146 = t10.*t65.*t135;
t148 = t17.*t65.*t135;
t175 = t109+t130;
t177 = t113+t128;
t178 = t115+t129;
t180 = t9.*t176;
t182 = t16.*t176;
t185 = -t9.*(t110-t129);
t186 = -t16.*(t110-t129);
t197 = t124+t133;
t239 = t147+t163;
t240 = t149+t150+3.0./1.0e+2;
t265 = t76.*t107.*t237;
t266 = t73.*t107.*t237;
t288 = t76.*t107.*t246;
t289 = t73.*t107.*t246;
t356 = t10.*t352;
t357 = t17.*t352;
t364 = t9.*t64.*t363;
t365 = t16.*t64.*t363;
t157 = -t148;
t179 = t9.*t175;
t181 = t16.*t175;
t183 = t9.*t177;
t184 = t16.*t177;
t188 = -t182;
t196 = t121+t138;
t198 = t123+t139;
t199 = t125+t137;
t201 = t9.*t197;
t203 = t16.*t197;
t206 = -t9.*(t122-t137);
t207 = -t16.*(t122-t137);
t241 = t146+t153+3.0./1.0e+2;
t273 = -t265;
t274 = -t76.*t107.*(t148+t10.*t65.*(t50+t75-t100-t116));
t275 = t76.*t107.*t239;
t276 = -t73.*t107.*(t148+t10.*t65.*(t50+t75-t100-t116));
t277 = t73.*t107.*t239;
t278 = t265./5.0e+1;
t279 = t266./5.0e+1;
t280 = t76.*t107.*(t148+t10.*t65.*(t50+t75-t100-t116));
t283 = t76.*t107.*t240;
t285 = t73.*t107.*t240;
t290 = t288./5.0e+1;
t291 = t289./5.0e+1;
t339 = t266+t288;
t343 = -t14.*t62.*(t265-t289);
t345 = -t7.*t62.*(t265-t289);
t347 = t7.*t62.*(t265-t289);
t362 = t14.*t62.*(t265-t289).*(-1.0);
t398 = t364+t368;
t399 = t365+t369;
t187 = -t181;
t200 = t9.*t196;
t202 = t16.*t196;
t204 = t9.*t198;
t205 = t16.*t198;
t209 = -t203;
t238 = t151+t157;
t257 = t180+t184;
t258 = t179+t186;
t259 = t183+t188;
t267 = -t10.*(t182-t183);
t268 = -t17.*(t182-t183);
t270 = -t10.*(t181+t9.*(t110-t129));
t271 = -t17.*(t181+t9.*(t110-t129));
t281 = -t275;
t284 = t76.*t107.*t241;
t286 = t73.*t107.*t241;
t292 = -t291;
t340 = t273+t289;
t341 = t14.*t62.*t339;
t342 = t7.*t62.*t339;
t208 = -t202;
t260 = t185+t187;
t261 = t10.*t257;
t262 = t17.*t257;
t263 = t10.*t258;
t264 = t17.*t258;
t293 = t201+t205;
t294 = t200+t207;
t295 = t204+t209;
t301 = -t10.*(t203-t204);
t302 = -t17.*(t203-t204);
t304 = -t10.*(t202+t9.*(t122-t137));
t305 = -t17.*(t202+t9.*(t122-t137));
t344 = -t341;
t346 = -t342;
t360 = t342.*1.0;
t371 = t277+t278+t283+t292;
t372 = t276+t278+t284+t292;
t373 = t279+t280+t286+t290;
t374 = t279+t281+t285+t290;
t269 = -t262;
t272 = -t264;
t296 = t206+t208;
t297 = t10.*t293;
t298 = t17.*t293;
t299 = t10.*t294;
t300 = t17.*t294;
t335 = t261+t268;
t337 = t263+t271;
t361 = -t360;
t375 = t7.*t62.*t371;
t376 = t7.*t62.*t372;
t377 = t14.*t62.*t371;
t378 = t14.*t62.*t372;
t379 = t7.*t62.*t373;
t380 = t7.*t62.*t374;
t381 = t14.*t62.*t373;
t382 = t14.*t62.*t374;
t391 = t46+t155+t160+t164+t343+t346;
t392 = t39+t158+t166+t167+t344+t347;
t303 = -t298;
t306 = -t300;
t336 = t267+t269;
t338 = t270+t272;
t348 = t297+t302;
t350 = t299+t305;
t383 = -t375;
t384 = -t376;
t385 = t375.*1.0;
t386 = t376.*1.0;
t389 = t381.*1.0;
t390 = t382.*1.0;
t393 = t15.*t63.*t392;
t394 = t8.*t63.*t392;
t396 = t8.*t63.*t391.*6.123233995736766e-17;
t397 = t15.*t63.*t391.*6.123233995736766e-17;
t400 = t378+t379-1.469576158976824e-18;
t401 = t377+t380-1.469576158976824e-18;
t414 = t174+t234+t236+t247+t249+t250+t255+t256+t361+t362;
t349 = t301+t303;
t351 = t304+t306;
t387 = -t385;
t388 = -t386;
t395 = -t393;
t402 = t382+t383+3.67394039744206e-19;
t403 = t381+t384+3.67394039744206e-19;
t404 = t8.*t63.*t400;
t405 = t8.*t63.*t401;
t406 = t15.*t63.*t400;
t407 = t15.*t63.*t401;
t415 = t119.*t414;
t416 = t120.*t414;
t419 = t334+t394+t397;
t408 = t8.*t63.*t402.*6.123233995736766e-17;
t409 = t8.*t63.*t403.*6.123233995736766e-17;
t410 = t15.*t63.*t402.*6.123233995736766e-17;
t411 = t15.*t63.*t403.*6.123233995736766e-17;
t417 = t224+t228+t251+t252+t387+t390+3.67394039744206e-19;
t418 = t224+t228+t251+t252+t388+t389+3.67394039744206e-19;
t420 = t9.*t64.*t419;
t421 = t16.*t64.*t419;
t422 = t48+t49+t159+t165+t168+t333+t395+t396;
t412 = -t410;
t413 = -t411;
t423 = t9.*t64.*t422;
t424 = t16.*t64.*t422;
t434 = t330+t407+t408+1.592040838891559e-18;
t435 = t330+t406+t409+1.592040838891559e-18;
t425 = -t423;
t426 = t327+t404+t413;
t427 = t327+t405+t412;
t436 = t16.*t64.*t434;
t437 = t16.*t64.*t435;
t438 = t9.*t64.*t434;
t439 = t9.*t64.*t435;
t440 = t420+t424;
t428 = t9.*t64.*t426;
t429 = t9.*t64.*t427;
t430 = t16.*t64.*t426;
t431 = t16.*t64.*t427;
t441 = t421+t425;
t442 = -t440.*(t298+t10.*(t203-t204));
t444 = t350.*t440;
t432 = -t428;
t433 = -t429;
t443 = t348.*t441;
t445 = -t441.*(t300+t10.*(t202+t9.*(t122-t137)));
t446 = t430+t439;
t447 = t431+t438;
t448 = t433+t436;
t449 = t432+t437;
J_end = reshape([-t416-t444+t445-t106.*t417-t335.*t447+(t262+t10.*(t182-t183)).*(t429-t436),-t415+t442+t443+t108.*t417-t337.*(t429-t436)-t447.*(t264+t10.*(t181+t9.*(t110-t129))),t189-t190+t242+t243-t375.*6.123233995736766e-17+t382.*6.123233995736766e-17+t103.*t447+t101.*(t429-t436)+2.249639673992787e-35,-t356+t359,-t357+t10.*(t314-t317),1.224646799147353e-17,t416+t444+t106.*t418+t335.*t446-(t262+t10.*(t182-t183)).*(t428-t437)+t441.*(t300+t10.*(t202+t9.*(t122-t137))),t415-t443+t440.*(t298+t10.*(t203-t204))-t108.*t418+t337.*(t428-t437)+t446.*(t264+t10.*(t181+t9.*(t110-t129))),-t189+t190-t242-t243+t376.*6.123233995736766e-17-t381.*6.123233995736766e-17-t103.*t446-t101.*(t428-t437)-2.249639673992787e-35,t356+t17.*(t314-t317),t357+t358,-1.224646799147353e-17,-t398.*(t262+t10.*(t182-t183))-t106.*t370+t335.*t399,t108.*t370+t337.*t398+t399.*(t264+t10.*(t181+t9.*(t110-t129))),t244+t15.*t63.*4.499279347985573e-34-t101.*t398-t103.*t399+t28.*t63.*t64.*7.960204194457796e-18-t32.*t63.*t64.*7.960204194457796e-18+1.124819836996393e-34,-t10.*t353-t17.*t355,t10.*t355-t17.*t353,6.123233995736766e-17,t86.*(-3.0e-2)-t87.*3.0e-2-t320.*(t262+t10.*(t182-t183))+t335.*(t230-t232),t85.*3.0e-2-t88.*3.0e-2+t320.*t337+(t230-t232).*(t264+t10.*(t181+t9.*(t110-t129))),-t101.*t320-t103.*(t230-t232)+1.83697019872103e-18,0.0,0.0,1.0,t16.*t64.*(t262+t10.*(t182-t183)).*(-1.3e+1./1.0e+2)-t9.*t64.*t335.*(1.3e+1./1.0e+2),t9.*t64.*(t264+t10.*(t181+t9.*(t110-t129))).*(-1.3e+1./1.0e+2)+t16.*t64.*t337.*(1.3e+1./1.0e+2),t9.*t64.*t103.*(1.3e+1./1.0e+2)-t16.*t64.*t101.*(1.3e+1./1.0e+2),0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,1.0],[6,6]);
