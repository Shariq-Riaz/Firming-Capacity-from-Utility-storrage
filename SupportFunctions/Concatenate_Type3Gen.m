function [Date Gen] = Concatenate_Type3Gen(Offset,Gen,Bus,T,Path)
%% Copyright Notice
% Copyright �2017. Shariq Riaz, Gregor Verbic. All Rights Reserved.
% Permission to use, copy, modify, and distribute this software and 
% its documentation for educational, research, and not-for-profit purposes,
% without fee and without a signed licensing agreement, is hereby granted,
% provided that the above copyright notice, this paragraph and the following
% two paragraphs appear in all copies, modifications, and distributions.
% Contact Gregor Verbic, School of Electrical and Information Engineering
% Centre for Future Energy Networks, J03 - Electrical Engineering Building,
% The University of Sydney, Ph +61 2 9351 8136, gregor.verbic@sydney.edu.au
% 
% IN NO EVENT SHALL AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT,
% INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
% LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS 
% DOCUMENTATION, EVEN IF AUTHORS HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
% DAMAGE.
% 
% AUTHORS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT 
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
% PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, 
% PROVIDED HEREUNDER IS PROVIDED "AS IS". AUTHORS HAS NO OBLIGATION TO 
% PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
%%
% function to cconcatenate RES data
% Output:
% Gen: Updated generation structure
% Inputs:
% Gen : Generation Matrix
% T_Hrz: number of hours for which the data is required
% Offset: adress of the specific date
% Path: Data folder path
% Author: Shariq Riaz CFEN SEIE USYD 
% Date:17/01/17
% % N_type3=find(Gen.Type==3);  % indexes of type 2 generators 
% % Gen.Trace.Type3=[];
% % Date=[];
% % for kk=1:length(N_type3)    
% %     switch char(Gen.Tech(N_type3(kk)))
% %         case 'CST'
% %             [Date.PV temp]=get_PV_Trace(Offset,T,char(Gen.Zone(kk)),Path);
% %             Gen.Trace.Type3=[Gen.Trace.Type3;Gen.Trace_Factor(kk)*temp];
% %         otherwise
% %             fprintf('###########Unsported technology###########\n');
% %     end
% % end
N_type3=find(Gen.Type==3);  % indexes of type 2 generators 
Gen.Trace.Type3=[];
Date=[];
for kk=1:length(N_type3)    
    switch char(Gen.Tech(N_type3(kk)))
        case 'CST'
            Trace_Name=Bus.Trace_Name.CST(find(not(cellfun('isempty',strfind(Bus.Name,char(Gen.Bus(N_type3(kk))))))));
            [Date.CST temp]=get_CST_Trace(Offset,Trace_Name,T,Path);
            Gen.Trace.Type3=[Gen.Trace.Type3;Gen.Trace_Factor((kk))*temp];
        otherwise
            fprintf('###########Unsported technology###########\n');
    end
end
end