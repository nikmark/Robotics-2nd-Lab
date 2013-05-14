function node_path = a_star_search(q_init,q_goal,g,map,n,node_info)

node_init = map(q_init(1),q_init(2));
node_goal = map(q_goal(1),q_goal(2));

if node_init == 0 || node_goal == 0
    
    node_path = [];
    return
    
end

grey   = zeros(1,n);
father = zeros(1,n);
step   = 1;

grey(node_init) = 1;
queue           = [ node_init; 0 ];

node       = queue(1,1);
queue(:,1) = [];
while node ~= node_goal
    
    children  = find(g(node,:));
    for i = 1:length(children)
        
        if grey(children(i)) == 0
            
            grey(children(i))   = 1;
            father(children(i)) = node;
            
            queue = enqueue(children(i),(step + H(q_goal,children(i),node_info)),queue);
            
        end
        
    end
    
    step = step + 1;
    
    if isempty(queue)
        
        node_path = [];
        return
        
    else
        
        node       = queue(1,1);
        queue(:,1) = [];
        
    end
    
end

node = node_goal;
node_path = node;
while node ~= node_init
    
    node = father(node);
    node_path = [ node node_path ];
    
end

end

function queue = enqueue(node,node_cost,queue)

l = length(queue(1,:));
for i = 1:l   
    if node_cost < queue(2,i)
        queue = [ queue(:,1:(i - 1)), [ node; node_cost ], queue(:,i:l) ];
        break
    end
end

if length(queue(1,:)) == l
    queue(:,end + 1) = [ node; node_cost ];
end

end

function h = H(q_goal,node,node_info)

m_i = node_info(node,1);
m_f = node_info(node,2);
n_i = node_info(node,3);
n_f = node_info(node,4);

q(1) = round((m_i + m_f - 1)/2);
q(2) = round((n_i + n_f - 1)/2);

h = norm(q_goal - q);

end