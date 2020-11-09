# docker run -it --rm -v /home/carlos/vrpsolver/demos/juliaPractice/:/juliaPractice bapdock
# include("/juliaPractice/MST.jl")

# Implementation of priority queue by means of max binary heap


function heapify_down!(heap::Vector{T}, i::Int64) where T
    left_child, right_child, largest = 2*i, 2*i + 1, i

    left_child ≤ length(heap) && heap[left_child] > heap[largest] ? (largest = left_child) : 0
    right_child ≤ length(heap) && heap[right_child] > heap[largest] ? (largest = right_child) : 0
    if largest != i
        heap[i], heap[largest] = heap[largest], heap[i]
        heapify_down!(heap, largest)
    end

    return nothing
end

function heapify_up!(heap::Vector{T}, i::Int64) where T
    parent = Integer(floor(i/2))

    if parent > 0 && heap[parent] < heap[i]
        heap[parent], heap[i] = heap[i], heap[parent]
        heapify_up!(heap, parent)
    end
    
    return nothing
end

function pq_push!(heap::Vector{T}, element::T) where T
    push!(heap, element)
    heapify_up!(heap, length(heap))
end

function pq_pop!(heap::Vector{T}) where T
    heap[1], heap[length(heap)] = heap[length(heap)], heap[1]
    element = pop!(heap)
    heapify_down!(heap, 1)

    return element
end

function build_max_heap!(heap::Vector{T}) where T
    for i = Integer(floor(length(heap)/2)):-1:1
        heapify_down!(heap, i)
    end
end

# disjoint set data structure

find!(disjoint_set::Vector{Int64}, x::Int64) = return disjoint_set[x]==x ? x : (disjoint_set[x] = find!(disjoint_set,disjoint_set[x]))

function union!(disjoint_set::Vector{Int64}, x::Int64, y::Int64)
    (x = find!(disjoint_set, x)) == (y = find!(disjoint_set, y)) && return nothing

    disjoint_set[y] = x
    return nothing  
end

# counting sort

key(x::Tuple{Int64,Int64,Int64}) = x[1]

function counting_sort!(input::Vector{T}, k::Int64) where T
    count = zeros(Int64, k + 1)

    for x in input
        count[key(x) + 1] += 1
    end

    total = 1
    for i=1:k+1
        count[i], total = total, count[i] + total
    end

    output = Vector{T}(undef, length(input))
    
    # println(count)
    
    for x in input
        output[count[key(x)+1]] = x
        count[key(x)+1] += 1
    end

    input .= output
end

# Instance data

mutable struct Vertex
    x::Float64
    y::Float64
    id::Int64
end

mutable struct Graph
    V::Vector{Vertex} # set of vertices
    cost::Array{Int64, 2} # arc cost
end

mutable struct Instance
    name::String
    G::Graph
    min_cost::Int64
    max_cost::Int64
end

function readInstance(file_path::String)
    file_lines = readlines(file_path)
    n = parse(Int64, split(file_lines[2], " ")[2])
    data = Instance(split(file_lines[1], " ")[2], Graph(Vector{Vertex}(undef, n), Array{Int64, 2}(undef, n, n)), typemax(Int64), 0)

    for i = 4:length(file_lines)-1
        v = split(file_lines[i])[2:3]
        data.G.V[i-3] = Vertex(parse(Float64, v[1]), parse(Float64, v[2]), i - 3)
    end

    for i = 1:n, j = i+1:n
        data.G.cost[i,i] = 0
        data.G.cost[j,i] = data.G.cost[i,j] = Integer(round(sqrt((data.G.V[i].x - data.G.V[j].x)^2 + (data.G.V[i].y - data.G.V[j].y)^2)))
        (data.G.cost[i,j] < data.min_cost) && global data.min_cost = data.G.cost[i,j]
        (data.G.cost[i,j] > data.max_cost) && global data.max_cost = data.G.cost[i,j]
    end
    return data
end

data = readInstance("/juliaPractice/instances/n1500C.txt")


function mst_kruskal_heap(data::Instance)
    G = data.G
    disjoint_set = collect(1:length(G.V))
    A = Vector{Tuple{Int64, Int64, Int64}}()

    pq = Vector{Tuple{Int64, Int64, Int64}}(undef, Integer((length(G.V)^2 - length(G.V))/2) )
    k = 1
    for i = 1:length(G.V), j = i+1:length(G.V)
        pq[k] = (-G.cost[i,j], i, j)
        k = k + 1
    end

    build_max_heap!(pq)

    tree_cost = 0
    while (length(A) != (length(data.G.V) - 1)) && !isempty(pq)
        (cost, i,j) = pq_pop!(pq)
        if find!(disjoint_set, i) != find!(disjoint_set, j)
            push!(A, (cost,i,j))
            union!(disjoint_set, i, j)
            tree_cost += cost
        end
    end
    tree_cost = -tree_cost
    
    return tree_cost
end

function mst_kruskal_count(data::Instance)
    G = data.G
    disjoint_set = collect(1:length(G.V))
    A = Vector{Tuple{Int64, Int64, Int64}}()

    pq = Vector{Tuple{Int64, Int64, Int64}}(undef, Integer((length(G.V)^2 - length(G.V))/2) )
    k = 1
    for i = 1:length(G.V), j = i+1:length(G.V)
        pq[k] = (G.cost[i,j], i, j)
        k = k + 1
    end

    counting_sort!(pq, data.max_cost)
    tree_cost = 0

    for edge in pq
        (cost, i,j) = edge
        if find!(disjoint_set, i) != find!(disjoint_set, j)
            push!(A, (cost,i,j))
            union!(disjoint_set, i, j)
            tree_cost += cost
        end
        (length(A) == (length(data.G.V) - 1)) && return tree_cost
    end    
end

@time (mst_kruskal_heap(data))
@time (mst_kruskal_count(data))

# pq = []
# for i = 1:10000
#     push!(pq, rand(Int)%1000 + 1000)
# end

# @time build_max_heap!(pq)

# for i = 1:length(pq)
#  #   print(pq_pop!(pq), " ")
# end

