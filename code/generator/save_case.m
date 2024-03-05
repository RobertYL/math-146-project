function save_case(cse,opts)
%SAVE_CASE Saves case to file
%   Saves case CSE to a file

    arguments
        cse Case
        opts.FileName string = strcat("case-",string(datetime("now",Format="MMddHHmmss")),".mat")
        opts.Location string = "cases"
    end

    save(fullfile(opts.Location,opts.FileName),"cse");
end

