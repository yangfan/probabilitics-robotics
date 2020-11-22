function p = log_odds_to_prob(l)
% Convert log odds l to the corresponding probability values p.
% l could be a scalar or a matrix.

% compute p.
p = 1 - 1 ./ (1 + exp(l));

end
