function Report=test_homework1_compare_pmks
folder=fileparts(mfilename('fullpath'));
M=load(fullfile(folder,'results_matlab','classroom_results.mat'));
S=load(fullfile(folder,'results_pmks_comparison','comparison_results.mat'));
count=0; max_initial=0;
for name=fieldnames(S.Tables)'
    t=S.Tables.(name{1});
    assert(isequal(t.MATLAB,M.Tables.(name{1}).MATLAB));
    assert(all(isfinite(t.PMKS)));
    count=count+height(t);
    if ~strcmp(name{1},'Static'), max_initial=max(max_initial,max(t.AbsDifference)); end
end
assert(count==95 && max_initial<.0021);
assert(numel(fieldnames(S.PMKS))==39);
assert(height(S.PMKSComparison)==67 && all(S.PMKSComparison.Samples==361));
assert(max(S.PMKSComparison.PeakScaledPercent,[],'omitnan')<3);
assert(~isfield(S.Tables,'Dynamic') && ~isfield(S.PMKS,'dynamic_T_diagnostic'));
assert(abs(S.Tables.Static.PMKS(end)+504.264)<1e-10);
assert(S.PMKS.omega_EF.vector(1,3)==1.083);
assert(S.PMKS.alpha_GFH.vector(1,3)==1.504);
for name={'A','D','G'}
    assert(all(S.PMKS.(['v_' name{1}]).vector==0,'all'));
    assert(all(S.PMKS.(['a_' name{1}]).vector==0,'all'));
end
for field=fieldnames(S.PMKS)'
    entry=S.PMKS.(field{1});
    assert(numel(entry.time)==361 && entry.time(1)==0 && abs(entry.time(end)-2.592)<1e-10);
    assert(all(diff(entry.time)>0) && all(isfinite(entry.vector),'all'));
    assert(contains(entry.source,'2026-09-11'));
end
code=fileread(fullfile(folder,'data_comparison_analysis.m'));
assert(all(double(code)<128));
for token={'readmatrix(','readtable(','webread(','system(','syms ','solve('}
    assert(~contains(code,token{1}));
end
assert(numel(dir(fullfile(folder,'results_pmks_comparison','*.png')))==7);
Report=struct('passed',true,'verified_PMKS_values',count,'native_series',39, ...
    'full_cycle_component_comparisons',67,'samples_per_series',361,'report_figures',7, ...
    'max_initial_kinematic_difference',max_initial, ...
    'max_peak_scaled_percent',max(S.PMKSComparison.PeakScaledPercent,[],'omitnan'));
fid=fopen(fullfile(folder,'results_pmks_comparison','test_report.json'),'w');
closer=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(Report,PrettyPrint=true));
disp(Report);
end
