function [matrix,Table] = access_data(T)

[numRows,numSequences] = size(T);

Table = T{1};

if numSequences > 1
    for i = 2:numSequences
        shift_value = table2array(T{i-1}(end,'Time'))+0.5;
        for k = 1:height(T{i}(:,'Time'))
            T{i}(k,'Time') = table(table2array(T{i}(k,'Time'))+shift_value);
        end
        Table = vertcat(Table,T{i});
    end
end

matrix = table2array(Table);
    
    
    

        
    
    
