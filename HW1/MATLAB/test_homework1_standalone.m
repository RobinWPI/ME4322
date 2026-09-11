function Report=test_homework1_standalone
% Verify the two scripts in an isolated folder.
source_folder=fileparts(mfilename('fullpath')); sandbox_folder=tempname; mkdir(sandbox_folder);
main_file=fullfile(sandbox_folder,'main.m');
comparison_file=fullfile(sandbox_folder,'data_comparison_analysis.m');
copyfile(fullfile(source_folder,'main.m'),main_file); hide_test_figures(main_file);
run_in_base(main_file);
result_file=fullfile(sandbox_folder,'results_matlab','classroom_results.mat');
M=load(result_file);
assert(~isfile(comparison_file));
assert(strcmp(M.P.load_model,'prescribed_force_at_H'));
assert(isequal(M.Q_history,repmat([0 -200 0],721,1)));
assert(abs(M.static_history(1,15)+504.372529988218)<1e-8);
assert(abs(M.dynamic_history(1,15)+47.174904869829)<1e-8);
assert(numel(dir(fullfile(sandbox_folder,'results_matlab','*.png')))==5);
copyfile(fullfile(source_folder,'data_comparison_analysis.m'),comparison_file);
hide_test_figures(comparison_file);
run_in_base(comparison_file);
comparison_result=fullfile(sandbox_folder,'results_pmks_comparison','comparison_results.mat');
S=load(comparison_result); assert(isequaln(M,load(result_file)));
assert(numel(fieldnames(S.PMKS))==39 && height(S.PMKSComparison)==67);
assert(numel(dir(fullfile(sandbox_folder,'results_pmks_comparison','*.png')))==7);
assert(~isfolder(fullfile(sandbox_folder,'pmks_web')));
count=0;
for name=fieldnames(S.Tables)'
    t=S.Tables.(name{1}); count=count+nnz(isfinite(t.PMKS));
    assert(isequal(t.MATLAB,M.Tables.(name{1}).MATLAB));
end
assert(count==95);
% Suppress repeat figure generation only in the isolated test copy.
text=fileread(comparison_file);
text=strrep(text,'plot_comparison(MATLAB,PMKS,show_plots,output_folder);','% Plotting already verified.');
write_copy(comparison_file,text);
for test=1:6
    invalid=M;
    switch test
        case 1, invalid.FirstPosition.Joints(1,1)=invalid.FirstPosition.Joints(1,1)+.01;
        case 2, invalid.time=invalid.time*1.1;
        case 3, invalid.FirstPosition.Joints(8,1)=invalid.FirstPosition.Joints(8,1)+.01;
        case 4, invalid.FirstPosition.COM(5,1)=invalid.FirstPosition.COM(5,1)+.01;
        case 5, invalid.P.mass(5)=invalid.P.mass(5)+1;
        case 6, invalid.FirstPosition.Q_static=[0 -201 0];
    end
    save(result_file,'-struct','invalid'); run_in_base(comparison_file);
    rejected=load(comparison_result);
    if test<=3
        assert(isempty(fieldnames(rejected.PMKS)));
        assert(all(isnan(rejected.Tables.AngularVelocity.PMKS)));
    elseif test==4
        assert(all(isnan(rejected.Tables.COMAcceleration.PMKS)));
        assert(all(isnan(rejected.Tables.Static.PMKS)));
        assert(all(isfinite(rejected.Tables.JointVelocity.PMKS)));
    else
        assert(all(isnan(rejected.Tables.Static.PMKS)));
        assert(all(isfinite(rejected.Tables.COMAcceleration.PMKS)));
    end
end
Report=struct('passed',true,'main_runs_without_comparison_script',true, ...
    'external_PMKS_files_required',false,'comparison_preserves_MATLAB_results',true, ...
    'reference_mismatch_cases_passed',6,'verified_PMKS_values',95, ...
    'full_cycle_component_comparisons',67,'report_figures',7,'sandbox_folder',sandbox_folder);
report_folder=fullfile(source_folder,'results_matlab');
if ~isfolder(report_folder), mkdir(report_folder); end
fid=fopen(fullfile(report_folder,'standalone_test_report.json'),'w'); assert(fid>0);
closer=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(Report,PrettyPrint=true)); disp(Report);
end

function run_in_base(file)
evalin('base',sprintf('run(''%s'');',strrep(file,'''','''''')));
evalin('base','close all;');
end

function hide_test_figures(file)
write_copy(file,strrep(fileread(file),'show_plots = true;','show_plots = false;'));
end

function write_copy(file,text)
fid=fopen(file,'w'); assert(fid>0); closer=onCleanup(@()fclose(fid)); fprintf(fid,'%s',text);
end
