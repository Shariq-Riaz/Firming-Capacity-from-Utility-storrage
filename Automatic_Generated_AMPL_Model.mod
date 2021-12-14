### SETS ###
set UGen;
set G_Syn in UGen;
set G_T1 in UGen;
param T>0;
set Time = 1..T;
set UBus;
set URegion;
set ULine;
	
### Type2 Generators Sets ###
set G_T2 in UGen;
	
### Utility Storage sets ###
set UStorage;
set Storage_Bus_links within (UStorage cross UBus);
	
### CROSS SETS LINKS ###
set Gen_Bus_links within (UGen cross UBus);
set Gen_Region_links within (UGen cross URegion);
set GenT1_Region_links within (G_T1 cross URegion);
set Line_end1_Bus_links within (ULine cross UBus);
set Line_end2_Bus_links within (ULine cross UBus);
set Bus_Region_links within (UBus cross URegion);


### Generator Cost Parameters ###
param C_Fix{UGen} >=0;
param C_Su{UGen} >=0;
param C_Sd{UGen} >=0;
param C_Var{UGen} >=0;
	
### Generator Parameters ###
param Max_pwr{UGen} >=0;
param Min_pwr{UGen} >=0;
param Ramp_up{UGen} >=0;
param Ramp_down{UGen} >=0;
param MUT{UGen}>=0;
param MDT{UGen}>=0;
param Units{UGen}>=0;
	
### Generator Initial Consitions ###
param Status_ini{UGen} >=0;
param Pwr_Gen_ini{UGen} >=0;
param MUT_ini{UGen,1..24} >=0;
param MDT_ini{UGen,1..24} >=0;
	
### Interconnector Parameters ###
param PwrLim{ULine} >=0;
param Susceptance{ULine};
	
### Demand Parameters ###
param Csm_Demand{UBus,Time} >=0;
param Psm_Demand{UBus,Time} >=0;
param Loss_factor >=0,<=1;
param PReserve_factor >=0,<=1;
param Base_power >=0;
	
### Type2 Generators Parameters ###
param Resource_trace_T2{G_T2,Time};
	
### Utility Storage parameters ###
param Chrg_rate_strg{UStorage} >=0;
param Dchrg_rate_strg{UStorage} >=0;
param Min_SOC_strg{UStorage} >=0;
param Max_SOC_strg{UStorage} >=0;
param Storage_eff{UStorage} >=0,<=1;
	
### Utility Storage inintial conditions ###
param Enrg_Strg_ini{UStorage} >=0;
	
### Demand responce parameters ###
param M_gp = 1e6;
param M_pv = 1e6;
param M_sp = 1e6;
param M_pl = 1e6;
param M_pu = 1e6;
param M_el = 1e6;
param M_eu = 1e6;
param PV_trace_DR{UBus,Time} >=0;
param Max_chrg_rate_bat{UBus} >=0;
param Max_dchrg_rate_bat{UBus} >=0;
param Min_SOC_bat{UBus} >=0;
param Max_SOC_bat{UBus} >=0;
param Bat_eff{UBus} >=0;
param alpha{UBus} >=0;
	
### Demand responce initial conditions ###
param Engy_bat_ini{UBus} >=0;
	
### Generator Decision Variables ###
var Status_var {g in UGen,Time} integer >=0,<=Units[g];
var S_Up_var {UGen,Time} integer >=0;
var S_Down_var {UGen,Time} integer >=0;
var Pwr_Gen_var {UGen,Time} >=0;
	
### Interconnector Decision Variables ###
var Pwr_line_var {ULine,Time};


### Bus Angle Decision Variables ###
var Angle_bus_var {UBus,Time} >= -3.1416,<=3.1416;


### Utility Storage decision variable ###
var Pwr_Strg_var {UStorage,Time};
var Pwr_SFirm_var {UStorage,Time} >=0;
var Enrg_Strg_var {UStorage,Time} >=0;
	
### Demand responce decision variables ###
var Pwr_pgp_var {UBus,Time} >=0;
var Pwr_pv_var {UBus,Time} >=0;
var Pwr_sp_var {UBus,Time} >=0;
var Engy_bat_var {UBus,Time} >=0;
var Pwr_batc_var {UBus,Time} >=0;
var Pwr_batd_var {UBus,Time} >=0;
	
### Slackness Variables ###
var lambda_pg_var {UBus,Time} ;
var lambda_pv_var {UBus,Time} ;
var lambda_e_var {UBus,Time} ;
var mu_gp_var {UBus,Time} >=0;
var mu_pl_var {UBus,Time} >=0;
var mu_pu_var {UBus,Time} >=0;
var mu_el_var {UBus,Time} >=0;
var mu_eu_var {UBus,Time} >=0;
var mu_pv_var {UBus,Time} >=0;
var mu_sp_var {UBus,Time} >=0;
	
### Orthognal maintaining Variables ###
var b_gp_var {UBus,Time} binary;
var b_pv_var {UBus,Time} binary;
var b_sp_var {UBus,Time} binary;
var b_pl_var {UBus,Time} binary;
var b_pu_var {UBus,Time} binary;
var b_eu_var {UBus,Time} binary;


### OBJECTIVE FUNCTION ###
minimize total_cost: sum {t in Time} sum {g in UGen}(C_Fix[g]*Status_var[g,t]
 + C_Su[g]*S_Up_var[g,t] + C_Sd[g]*S_Down_var[g,t] + C_Var[g]*Pwr_Gen_var[g,t] );


### Balance Constraint ###
subject to Balance {n in UBus, t in Time}: sum{(g,n) in Gen_Bus_links} Pwr_Gen_var[g,t]
 + sum{(l2,n) in Line_end2_Bus_links}(Pwr_line_var[l2,t]) 
== Csm_Demand[n,t] + Loss_factor*Csm_Demand[n,t] 
 + sum{(l1,n) in Line_end1_Bus_links}(Pwr_line_var[l1,t])
 + sum{(s,n) in Storage_Bus_links}(Pwr_Strg_var[s,t])
 +  Pwr_pgp_var[n,t] + Loss_factor*Pwr_pgp_var[n,t];


### Active Power Reserve Constraint ###
subject to Power_Reserve {r in URegion, t in Time}: sum{(g,r) in GenT1_Region_links} Status_var[g,t]*Max_pwr[g]
 - sum{(g,r) in GenT1_Region_links} Pwr_Gen_var[g,t] 
>= sum{(n,r) in Bus_Region_links} PReserve_factor*Csm_Demand[n,t]
 + sum{(n,r) in Bus_Region_links} PReserve_factor*Pwr_pgp_var[n,t];


### Stable Limit of Generators Constraints ###
subject to Gen_max_pwr {g in G_Syn,t in Time}: Pwr_Gen_var[g,t] <= Max_pwr[g]*Status_var[g,t];
subject to Gen_min_pwr {g in G_Syn,t in Time}: Min_pwr[g]*Status_var[g,t] <= Pwr_Gen_var[g,t];
	
### Integer variable linking Constraint ###
subject to On_Off {g in G_Syn,t in 2..T}: S_Up_var[g,t] - S_Down_var[g,t] == Status_var[g,t] - Status_var[g,t-1];
subject to On_Off_initial {g in G_Syn}: S_Up_var[g,1] - S_Down_var[g,1] == Status_var[g,1] - Status_ini[g];
	
### Generator Ramping Constraints ###
subject to ramp_up {g in G_Syn, t in 2..T}:Ramp_up[g]<Max_pwr[g] ==> Pwr_Gen_var[g,t] - Pwr_Gen_var[g,t-1] <= Status_var[g,t]*Ramp_up[g];
subject to ramp_up_initial {g in G_Syn}:Ramp_up[g]<Max_pwr[g] ==> Pwr_Gen_var[g,1] - Pwr_Gen_ini[g] <= Status_var[g,1]*Ramp_up[g];
subject to ramp_down {g in G_Syn, t in 2..T}:Ramp_down[g]<Max_pwr[g] ==> Pwr_Gen_var[g,t-1] - Pwr_Gen_var[g,t] <= Status_var[g,t-1]*Ramp_down[g];
subject to ramp_down_initial {g in G_Syn}:Ramp_down[g]<Max_pwr[g] ==> Pwr_Gen_ini[g] - Pwr_Gen_var[g,1] <= Status_ini[g]*Ramp_down[g];
	
### Generator Minimum Up/Down Time Constraints ###
subject to min_up_Time {g in G_Syn, t in MUT[g]..T}:MUT[g]>1 ==> Status_var[g,t]
 >= sum{t1 in 0..MUT[g]-1} S_Up_var[g,t-t1]  ;
subject to min_up_Time_ini {g in G_Syn, t in 1..MUT[g]-1}:MUT[g]>1 ==> Status_var[g,t]
 >= sum{t1 in 0..t-1} S_Up_var[g,t-t1] + MUT_ini[g,t] ;
	
subject to min_down_Time {g in G_Syn, t in MDT[g]..T}:MDT[g]>1 ==> Status_var[g,t]
 <= Units[g] - sum{t1 in 0..MDT[g]-1} S_Down_var[g,t-t1] ;
subject to min_down_Time_ini {g in G_Syn, t in 1..MDT[g]-1}:MDT[g]>1 ==> Status_var[g,t]
 <= Units[g] - sum{t1 in 0..t-1} S_Down_var[g,t-t1] - MDT_ini[g,t] ;


### Maximum limit on ON units ###
subject to max_ONunits {g in UGen, t in Time}:
Status_var[g,t] <= Units[g];
### Thermal limits of interconnect Constraints ###
subject to thermal_limit_ub {l in ULine, t in Time}: Pwr_line_var [l,t] <= PwrLim[l];
subject to thermal_limit_lb {l in ULine, t in Time}: -PwrLim[l] <= Pwr_line_var[l,t] ;
	
### AC line angle stablility ###
subject to angle_limit {l in ULine, t in Time}: Pwr_line_var[l,t] == Base_power*Susceptance[l]*
 (sum{(l,n1) in Line_end1_Bus_links} Angle_bus_var[n1,t]- sum{(l,n2) in Line_end2_Bus_links} Angle_bus_var[n2,t]);


### Type2 Power Limit ###
subject to Resource_availability_T2 {g in G_T2, t in Time}: Pwr_Gen_var[g,t] <= Status_var[g,t]*Resource_trace_T2[g,t];


subject to G_T2_min_pwr {g in G_T2, t in Time}: Status_var[g,t]*Min_pwr[g]  <= Pwr_Gen_var[g,t];


subject to Firm_Capacity {t in Time}: 
sum{g in G_T2} Status_var[g,t]*Resource_trace_T2[g,t] + Status_var['G0015',t]*Max_pwr['G0015'] + 
sum{s in UStorage} Pwr_SFirm_var[s,t] >= 750;


subject to PSFirm {s in UStorage, t in Time}: Pwr_SFirm_var[s,t] <= Dchrg_rate_strg[s] + Pwr_Strg_var [s,t];
subject to ESFirm {s in UStorage, t in 2..T}: Pwr_SFirm_var[s,t] <= 
 Storage_eff[s]*Enrg_Strg_var[s,t-1];
subject to ESFirm_ini {s in UStorage}: Pwr_SFirm_var[s,1] <= Storage_eff[s]*Enrg_Strg_ini[s];


### Utility Storage Energy Balance Constraint ###
subject to Storage_energy_balance {s in UStorage, t in 2..T}: Enrg_Strg_var [s,t] 
= Storage_eff[s]*Enrg_Strg_var [s,t-1] + Pwr_Strg_var[s,t];
subject to Storage_energy_balance_Initial {s in UStorage}: Enrg_Strg_var [s,1] 
= Storage_eff[s]*Enrg_Strg_ini[s] + Pwr_Strg_var [s,1];
	
### Charge/Discharge rate Constraints ###
subject to Charge_rate_Storage {s in UStorage, t in Time}:  Pwr_Strg_var [s,t]  <= Chrg_rate_strg[s];
subject to Dcharge_rate_Storage {s in UStorage, t in Time}:  Pwr_Strg_var [s,t]  >= -Dchrg_rate_strg[s];
	
### Storage SOC Constraints ###
subject to Min_SOC_Strg {s in UStorage, t in Time}: Enrg_Strg_var [s,t] >= Min_SOC_strg[s];
subject to Max_SOC_Strg {s in UStorage, t in Time}: Enrg_Strg_var [s,t] <= Max_SOC_strg[s];


### DR Equality Constraints ###
## KKT Constraints ##
subject to KKT_pgp {p in UBus, t in Time}: lambda_pg_var[p,t] - mu_gp_var[p,t]  == -1;
subject to KKT_pbatc {p in UBus, t in Time}: -lambda_pg_var[p,t] - lambda_e_var[p,t] + mu_pu_var[p,t]  == 0;
subject to KKT_pbatd {p in UBus, t in Time}: lambda_pg_var[p,t] + lambda_e_var[p,t] - mu_pl_var[p,t]  == 0;
subject to KKT_ppv {p in UBus, t in Time}: lambda_pg_var[p,t] + lambda_pv_var[p,t] - mu_pv_var[p,t] == 0;
subject to KKT_pspill {p in UBus, t in Time}: lambda_pv_var[p,t] - mu_sp_var[p,t] == 0;
subject to KKT_ebat {p in UBus, t in 1..(T-1)}: lambda_e_var[p,t] - Bat_eff[p]*lambda_e_var[p,t+1] + mu_eu_var[p,t] == 0;
	
## System Constraints ##
subject to Grid_bus_bal {p in UBus, t in Time}: Pwr_pgp_var[p,t] + Pwr_pv_var[p,t] - Pwr_batc_var[p,t] + Pwr_batd_var[p,t] == Psm_Demand[p,t];
subject to PV_bus_bal {p in UBus, t in Time}: Pwr_pv_var[p,t] + Pwr_sp_var[p,t]  == PV_trace_DR[p,t];
subject to Battery_SOC {p in UBus, t in 2..T}:  Engy_bat_var[p,t] - Bat_eff[p]*Engy_bat_var[p,t-1] - Pwr_batc_var[p,t] + Pwr_batd_var[p,t]  == 0 ;
subject to Battery_SOC_Initial {p in UBus}:  Engy_bat_var[p,1] - Bat_eff[p]*Engy_bat_ini[p] - Pwr_batc_var[p,1] + Pwr_batd_var[p,1]  == 0 ;


## Inequality Constraints ##
## Orthogonal Constraints ##
subject to mu_gp_perp_pgp_A {p in UBus, t in Time}:
		 Pwr_pgp_var[p,t] <= M_gp*b_gp_var[p,t];
subject to mu_gp_perp_pgp_B {p in UBus, t in Time}:
		 mu_gp_var[p,t] <= M_gp * (1 - b_gp_var[p,t]) ;
	
subject to mu_pv_perp_ppv_A {p in UBus, t in Time}:
		 Pwr_pv_var[p,t] <= M_pv*b_pv_var[p,t];
subject to mu_pv_perp_ppv_B {p in UBus, t in Time}:
		 mu_pv_var[p,t] <= M_pv * (1 - b_pv_var[p,t]) ;
	
subject to mu_sp_perp_psp_A {p in UBus, t in Time}:
		 Pwr_sp_var[p,t] <= M_sp*b_sp_var[p,t];
subject to mu_sp_perp_psp_B {p in UBus, t in Time}:
		 mu_sp_var[p,t] <= M_sp * (1 - b_sp_var[p,t]) ;
	
subject to mu_pl_perp_pb_A {p in UBus, t in Time}:
		 Pwr_batd_var[p,t] <= M_pl*b_pl_var[p,t] + Max_dchrg_rate_bat[p];
subject to mu_pl_perp_pb_B {p in UBus, t in Time}:
		 mu_pl_var[p,t] <= M_pl * (1 - b_pl_var[p,t]) ;
	
subject to mu_pl_perp_pb_C {p in UBus, t in Time}:
		 Pwr_batd_var[p,t] <= Max_dchrg_rate_bat[p] ;
	
subject to mu_pu_perp_pb_A {p in UBus, t in Time}:
		 Pwr_batc_var[p,t] >= M_pu*b_pu_var[p,t] + Max_chrg_rate_bat[p];
subject to mu_pu_perp_pb_B {p in UBus, t in Time}:
		 mu_pu_var[p,t] <= M_pu * (1 - b_pu_var[p,t]) ;
	
subject to mu_pu_perp_pb_C {p in UBus, t in Time}:
		 Pwr_batc_var[p,t] <= Max_chrg_rate_bat[p] ;
	
subject to mu_eu_perp_eb_A {p in UBus, t in Time}:
		 -Engy_bat_var[p,t] <= M_eu*b_eu_var[p,t] + Max_SOC_bat[p];
subject to mu_eu_perp_eb_B {p in UBus, t in Time}:
		 mu_eu_var[p,t] <= M_eu * (1 - b_eu_var[p,t]) ;
	
subject to mu_eu_perp_eb_C {p in UBus, t in Time}:
		 Engy_bat_var[p,t] <= Max_SOC_bat[p] ;
	
