% calculates the number of non-zeros of a graph
% Actually, it is an upper bound, as duplicate edges might be counted several times

function nnz = nnz_of_graph(g)

nnz = 0;


 

% elements along the diagonal%

%for [value, key] = g.idLookup
%  nnz = nnz + value.dimension^2;
%end

fields = fieldnames(g.idLookup);
for i = 1:numel(fields)
    dim = g.idLookup.(fields{i}).dimension;
    nnz = nnz + dim^2;
end
       

% off-diagonal elements
for eid = 1:length(g.edges)
  edge = g.edges(eid);
  if (strcmp(edge.type, 'P') ~= 0)
    nnz = nnz + 2 * 9;
  elseif (strcmp(edge.type, 'L') ~= 0)
    nnz = nnz + 2 * 6;
  end
end

end
