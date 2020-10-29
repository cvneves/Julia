# docker run -it --rm -v /home/carlos/vrpsolver/demos/juliaPractice/:/juliaPractice bapdock
# include("/juliaPractice/TSP.jl")

mutable struct Vertex
    x::Float64
    y::Float64
    id::Int64
end

mutable struct Graph
    V::Vector{Vertex} # set of vertices
    cost::Dict{Tuple{Vertex, Vertex}, Float64} # arc cost
end

function generateRandomGraph(n::Int64, quadrantSize::Float64)
    G = Graph(Vector{Vertex}(), Dict())
    for i=1:n
        v = Vertex(rand() * quadrantSize, rand() * quadrantSize, i)
        push!(G.V, v)
    end
    for i=1:n, j=1:n
        if i != j 
            G.cost[Tuple([G.V[i] G.V[j]])] = round(sqrt((G.V[i].x - G.V[j].x)^2 + (G.V[i].y - G.V[j].y)^2))
        end
    end
    return G
end

mutable struct Solution
    sequence::Vector{Vertex}
    cost::Float64
end

Base.copy(s::Solution) = Solution(s.sequence, s.cost)

function generateRandomSolution(G::Graph)
    V = copy(G.V)

    s = Solution(Vector{Vertex}(), 0.0)

    while !isempty(V)
        idx = rand(1:length(V))
        V[length(V)], V[idx] = V[idx], V[length(V)]
        v = pop!(V)
        push!(s.sequence, v)
    end

    push!(s.sequence, s.sequence[1])

    calculateCost!(s)

    return s
end

function calculateCost!(s::Solution)
    s.cost = 0

    for i = 1:length(s.sequence)-1
        s.cost = s.cost + G.cost[Tuple([s.sequence[i] s.sequence[i+1]])]
    end

    return s.cost
end

function showSolution(s::Solution)
    for i = 1:length(s.sequence)
        print(s.sequence[i].id, " ")
    end
    println()
end

# Best Double-Bridge
const G = generateRandomGraph(100, 100.0)
const s = generateRandomSolution(G)
const n = length(G.V)

const best_reach = Dict{Char, Array{Float64,2}}()
const best_cross = Dict{Char, Array{Float64,2}}()
best_reach['c'] = Array{Float64,2}(undef, n,n)
best_reach['d'] = Array{Float64,2}(undef, n,n)
best_cross['c'] = Array{Float64,2}(undef, n,n)
best_cross['d'] = Array{Float64,2}(undef, n,n)

best_t = Inf64
const cost = Dict{Tuple{Char, Int64, Int64}, Float64}()
const pred_reach = Dict{Char, Array{Any,2}}()
const pred_cross = Dict{Char, Array{Any,2}}()
pred_reach['c'] = Array{Any,2}(undef, n,n)
pred_reach['d'] = Array{Any,2}(undef, n,n)
pred_cross['c'] = Array{Any,2}(undef, n,n)
pred_cross['d'] = Array{Any,2}(undef, n,n)

pred_t = undef
move_type = undef


function compute_cost(x::Char, i::Int64, j::Int64)
    # if haskey(cost, (x,i,j))
    #     return cost[(x,i,j)]
    # end

    if x == 'd'
        cost[(x,i,j)] = -G.cost[(s.sequence[i], s.sequence[i+1])] - G.cost[(s.sequence[j], s.sequence[j+1])] + G.cost[(s.sequence[i], s.sequence[j+1])] + G.cost[(s.sequence[i+1], s.sequence[j])]
    end
    if x == 'c'
        cost[(x,i,j)] = -G.cost[(s.sequence[i], s.sequence[i+1])] - G.cost[(s.sequence[j], s.sequence[j+1])] + G.cost[(s.sequence[i], s.sequence[j])] + G.cost[(s.sequence[i+1], s.sequence[j+1])]
    end

    return cost[(x,i,j)]
end

# 3.3.1. Best cross update (for j ≤ n - 1)
function best_cross_update(i::Int64, j::Int64, x::Char)
    if best_reach[x][i - 1, j] < best_cross[x][i-1, j-1]
        best_cross[x][i-1, j] = best_reach[x][i-1, j]
        pred_cross[x][i-1,j] = (pred_reach[x][i-1, j], j)
    else
        best_cross[x][i-1, j] = best_cross[x][i-1, j-1]
        pred_cross[x][i-1, j] = pred_cross[x][i-1, j-1]
    end
end

# 3.3.2. Best reach update (for j ≤ n - 1)
function best_reach_update(i::Int64, j::Int64, x::Char)
    if compute_cost(x, i, j) < best_reach[x][i - 1, j]
        best_reach[x][i, j] = compute_cost(x, i, j)
        pred_reach[x][i, j] = i
    else
        best_reach[x][i, j] = best_reach[x][i - 1, j]
        pred_reach[x][i, j] = pred_reach[x][i - 1, j]
    end
end

# 3.3.3. Best terminal node update 
function best_terminal_node_update(i::Int64, j::Int64, x::Char, y::Char)
    if compute_cost(x, i, j) + best_cross[y][i-1,j-1] < best_t
        global best_t = compute_cost(x, i, j) + best_cross[y][i-1,j-1]
        global pred_t = (pred_cross[y][i-1,j-1], (i,j))
        global move_type = (x,y)
    end
end

# 3.3.4. Simple terminal node update (in case a simple 2-opt may be best)
function simple_terminal_node_update(i::Int64, j::Int64)
    if compute_cost('c', i, j) < best_t
        global best_t = compute_cost('c',i,j)
        global pred_t = ((0,0), (i,j))
        global move_type = ('c','c')
    end
end

function shortest_path_method()    

    # Step 1. (Initialization). best_t = ∞
    global best_t = Inf64
    for j = 3:n-1, x in ['c', 'd']
       best_reach[x][1,j] = compute_cost(x, 1, j)
       pred_reach[x][1, j] = 1
    end

    # Step 2. (Main Step).
    for i = 2:(n - 2)
        for x in ['c', 'd']
            best_cross[x][i-1, i+1] = best_reach[x][i-1, i+1]
            pred_cross[x][i-1, i+1] = (pred_reach[x][i-1,i+1], i+1)
        end
        for j = (i+2):n
            if j <= n-1
                for x in ['c', 'd']
                    best_cross_update(i,j,x)
                    best_reach_update(i,j,x)
                end
            end
            
            for (x,y) in [('d','d'), ('d','c'), ('c','d')]
                best_terminal_node_update(i,j,x,y)
            end               
            simple_terminal_node_update(i,j)
        end
    end

    # Step 3. (Solution Recovery)

    if best_t < 0

        println(best_t, ", ",move_type,", ", pred_t)
        println("Previous cost: ",s.cost)
        
        ((i1,j1),(i2,j2)) = pred_t
        (x,y) = move_type

        showSolution(s)
        if x == 'c' && y == 'c'
            s.sequence[i2+1:j2] = reverse!(s.sequence[i2+1:j2])
        elseif x == 'd' && y == 'd'
            s.sequence[i1+1:j2] = circshift(s.sequence[i1+1:j2], j2 - j1)
            s.sequence[i1+1+j2-j1:j2] = circshift(s.sequence[i1+1+j2-j1:j2], j2 - (i1+j2 - j1 + i2-i1))
        else
            s.sequence[i1+1:j2] = circshift(s.sequence[i1+1:j2], j2 - j1)
            s.sequence[i1+1+j2-j1:j2] = circshift(s.sequence[i1+1+j2-j1:j2], j2 - (i1+j2 - j1 + i2-i1))
            if x == 'c'
                subseq_1_end, subseq_2_end = i1 + j2 - j1, j2
                s.sequence[subseq_1_end+1:subseq_2_end] = reverse!(s.sequence[subseq_1_end+1:subseq_2_end])
            else
                subseq_1_end, subseq_2_end = i1, j2 - i2 + i1
                s.sequence[subseq_1_end+1:subseq_2_end] = reverse!(s.sequence[subseq_1_end+1:subseq_2_end])
            end
        end

        showSolution(s)

        calculateCost!(s)
        println("Current cost: ",s.cost)

    end
    # println(compute_cost(y,i1,j1) + compute_cost(x,i2,j2))

end


function main()
    
end