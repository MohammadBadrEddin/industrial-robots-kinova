% --- DIAGNOSE: Geometrie aus MATLAB-Modell extrahieren ---
  q_zero = zeros(7, 1);
  fprintf('\nBody-Namen:\n');
  for i = 1:gen3.NumBodies
      fprintf('  %d: %s\n', i, gen3.Bodies{i}.Name);
  end

  fprintf('\nRelative Transformationen bei q=0 (Frame i-1 -> Frame i):\n');
  T_prev = eye(4);
  for i = 1:gen3.NumBodies
      T_i   = getTransform(gen3, q_zero, gen3.Bodies{i}.Name);
      T_rel = T_prev \ T_i;
      fprintf('\n--- %s ---\n', gen3.Bodies{i}.Name);
      disp(T_rel)
      T_prev = T_i;
  end
  % --- Ende Diagnose ---
