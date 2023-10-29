clc
more off;
clear all;
close all;

addpath('tools');

% load the graph into the variable g
% only leave one line uncommented

for set = 1:4 

    % data sets (both real and simulated)
    if (set == 1)
        load ../data/simulation-pose-pose.mat
    elseif (set == 2)
        load ../data/intel.mat
    elseif (set == 3)
        load ../data/simulation-pose-landmark.mat
    else
        load ../data/dlr.mat
    end
    
    % the number of iterations
    numIterations = 100;
    
    % maximum allowed dx
    EPSILON = 10^-4;
    
    % Error
    err = compute_global_error(g);
    allErr(1,set) = err ;
    
    % plot the initial state of the graph
    plot_graph(g, 0, set);
    
    fprintf('Initial error %f\n', compute_global_error(g));
    
    % iterate
    for i = 1:numIterations 
      fprintf('Performing iteration %d\n', i);
    
      % compute the incremental update dx of the state vector
      dx = linearize_and_solve(g);
    
    
      % (TODO) apply the solution to the state vector g.x
        
      g.x = g.x + dx;
    
    
      % plot the current state of the graph
      plot_graph(g, i, set);
    
      % Compute the global error given the current graph configuration
      err = compute_global_error(g);
      allErr(i+1,set) = err ;

      % Print current error
      fprintf('Current error %f\n', err);
    
      % termination criterion  
      if(max(abs(dx)) < EPSILON)
	    disp('Converged!!');
	    break;
      end
    
    end
    
    fprintf('Done!\nFinal error %f\n', err);

end

f = figure ;
subplot(2,2,1)
plot(1:10 , allErr([1:10],1))
title('simulation-pose-pose')
xlabel('Iteration')
ylabel('Error')

subplot(2,2,2)
plot(1:6 , allErr([1:6],2))
title('intel')
xlabel('Iteration')
ylabel('Error')

subplot(2,2,3)
plot(1:5 , allErr([1:5],3))
title('simulation-pose-landmark')
xlabel('Iteration')
ylabel('Error')

subplot(2,2,4)
plot(1:11 , allErr([1:11],4))
title('dlr')
xlabel('Iteration')
ylabel('Error')

exportgraphics(f,['../plots/errors' '.jpg'])