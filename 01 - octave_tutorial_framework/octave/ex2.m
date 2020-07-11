
h1 = [1; 1; pi()/2];

M1 = [
  eye(length(x)), [2; 0];
  zeros(1, length(x)), 1
]

h2 = M1*h1;

display(t2v(h2));
