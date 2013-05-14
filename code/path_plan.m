function path = path_plan(q_init,q_goal,w,w_res)

res = 16*w_res*2;
s = size(w);
node_path = [];

while isempty(node_path) && res > w_res
    
    res = res/2;
    [ map, n, node_info ] = cells_decomposition([ 1 s(1) 1 s(2) ],zeros(s(1),s(2)),0,[],w,w_res,res);
    
    g = graph(map,n,node_info);
    
    node_path = a_star_search((fix(q_init/w_res) + 1),(fix(q_goal/w_res) + 1),g,map,n,node_info);
    
    path = q_init;
    dir_last = 0;
    
    for i = 2:length(node_path)
        [ channel, dir ] = f_channel(node_path(i - 1),node_path(i),map,node_info);
        switch dir
            case 1
                if dir_last == 2
                    path = [ path; w_res*center(node_path(i - 1),node_info) ];
                end
            case 2
                if dir_last == 1
                    path = [ path; w_res*center(node_path(i - 1),node_info) ];
                end
            case 3
                if dir_last == 4
                    path = [ path; w_res*center(node_path(i - 1),node_info) ];
                end
            case 4
                if dir_last == 3
                    path = [ path; w_res*center(node_path(i - 1),node_info) ];
                end
        end
        path = [ path; w_res*channel ];
        dir_last = dir;
    end
    path = [ path; q_goal ];
end

if ~isempty(node_path)
    
    s = size(path);
    x = path(:,1); y = path(:,2);
    t = zeros(1,s(1));
    
    for i = 2:length(x)
        t(i) = t(i - 1) + norm([ x(i) y(i) ] - [ x(i - 1) y(i - 1) ]);
    end

    tt = t(1):w_res:t(end);
    xx = spline(t,x,tt);
    yy = spline(t,y,tt);

    path = [ xx' yy' ];
else
    path = [];
end

end

function [ map, n, node_info ] = cells_decomposition(node,map,n,node_info,w,w_res,res)

m_i = node(1);
m_f = node(2);
n_i = node(3);
n_f = node(4);

lab = label(node,w);

switch lab
    case 'EMPTY'
        n = n + 1;
        map(m_i:m_f,n_i:n_f) = n;
        node_info = [ node_info; node ];
    case 'MIXED'
        if m_f - m_i > round(res/w_res)

            node_1 = [ m_i, fix((m_i + m_f - 1)/2), n_i, fix((n_i + n_f - 1)/2) ];
            node_2 = [ m_i, fix((m_i + m_f - 1)/2), fix((n_i + n_f + 1)/2), n_f ];
            node_3 = [ fix((m_i + m_f + 1)/2), m_f, n_i, fix((n_i + n_f - 1)/2) ];
            node_4 = [ fix((m_i + m_f + 1)/2), m_f, fix((n_i + n_f + 1)/2), n_f ];
            
            [ map, n, node_info ] = cells_decomposition(node_1,map,n,node_info,w,w_res,res);
            [ map, n, node_info ] = cells_decomposition(node_2,map,n,node_info,w,w_res,res);
            [ map, n, node_info ] = cells_decomposition(node_3,map,n,node_info,w,w_res,res);
            [ map, n, node_info ] = cells_decomposition(node_4,map,n,node_info,w,w_res,res);
        end
end

end

function lab = label(node,w)

m_i = node(1);
m_f = node(2);
n_i = node(3);
n_f = node(4);

if isempty(find(w(m_i:m_f,n_i:n_f),1))
    lab = 'FULL';
elseif isempty(find(-w(m_i:m_f,n_i:n_f) + 1,1))
    lab = 'EMPTY';
else
    lab = 'MIXED';
end
end

function [ channel, dir ] = f_channel(node_1,node_2,map,node_info)

s = size(map);

m_i = node_info(node_1,1);
m_f = node_info(node_1,2);
n_i = node_info(node_1,3);
n_f = node_info(node_1,4);

if m_i > 1
    border = find(map((m_i - 1),n_i:n_f) == node_2) + n_i - 1;
    if ~isempty(border)
        channel = [ (m_i - 1) (mean(border) - 1/2) ];
        dir = 1;
        return
    end
end

if m_f < s(1)
    border = find(map((m_f + 1),n_i:n_f) == node_2) + n_i - 1;
    if ~isempty(border)
        channel = [ m_f (mean(border) - 1/2) ];
        dir = 2;
        return
    end
end

if n_i > 1
    border = find(map(m_i:m_f,(n_i - 1)) == node_2) + m_i - 1;
    if ~isempty(border)
        channel = [ (mean(border) - 1/2) (n_i - 1) ];
        dir = 3;
        return
    end
end

if n_f < s(2)
    border = find(map(m_i:m_f,(n_f + 1)) == node_2) + m_i - 1;
    if ~isempty(border)
        channel = [ (mean(border) - 1/2) n_f ];
        dir = 4;
        return
    end
end

end

function q = center(node,node_info)

m_i = node_info(node,1);
m_f = node_info(node,2);
n_i = node_info(node,3);
n_f = node_info(node,4);

q(1) = (m_i + m_f - 1)/2;
q(2) = (n_i + n_f - 1)/2;

end