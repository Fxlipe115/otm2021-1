using JuMP
using Gurobi
using Debugger

function edgesLeaving(vertex, edges)
    delta = zeros(Int64, 0)
    for edge in edges
        if edge[1] == vertex
            append!(delta, edge[2])
        end
    end
    return delta
end

function edgesArriving(vertex, edges)
    delta = zeros(Int64, 0)
    for edge in edges
        if edge[2] == vertex
            append!(delta, edge[1])
        end
    end
    return delta
end

function main()
    timeLimit = parse(Float64, ARGS[1])

    numberOfVertices = 0
    numberOfEdges = 0
    pathOrigin = 0
    pathDestination = 0

    (numberOfVertices, numberOfEdges, pathOrigin, pathDestination) = split(readline())
    numberOfVertices = parse(Int64, numberOfVertices)
    numberOfEdges = parse(Int64, numberOfEdges)
    pathOrigin = parse(Int64, pathOrigin)
    pathDestination = parse(Int64, pathDestination)

    # edges
    edges = fill((0, 0), numberOfEdges * 2) # to make graph undirected

    # edges distances
    distance = fill(0, numberOfEdges * 2)

    for edgeId = 1 : numberOfEdges
        (u, v, d) = split(readline())
        u = parse(Int64, u)
        v = parse(Int64, v)
        d = parse(Int64, d)
        
        edges[edgeId] = (u, v)
        edges[numberOfEdges+edgeId] = (v, u)
        distance[edgeId] = d
        distance[numberOfEdges+edgeId] = d
    end

    model = Model(Gurobi.Optimizer)
    set_optimizer_attribute(model, "TimeLimit", timeLimit)
    set_optimizer_attribute(model, "Threads", 1)
    set_optimizer_attribute(model, "NodefileStart", 0.5) # value recommended by the documentation

    infinite = maximum(distance)

    @variable(model, p >= 0)
    @variable(model, M >= 0)
    @variable(model, step[1:numberOfVertices] >= 0)
    @variable(model, x[1:numberOfVertices, 1:numberOfVertices], Bin)

    @objective(model, Min, p)

    for i = 1:numberOfVertices
        deltaPlus = edgesLeaving(i, edges)
        deltaMinus = edgesArriving(i, edges)
        if i == pathOrigin
            @constraint(model, sum(x[i,j] for j in deltaPlus) 
                             - sum(x[j,i] for j in deltaMinus) == 1)
        elseif i == pathDestination
            @constraint(model, sum(x[i,j] for j in deltaPlus) 
                             - sum(x[j,i] for j in deltaMinus) == -1)
        else
            @constraint(model, sum(x[i,j] for j in deltaPlus) 
                             - sum(x[j,i] for j in deltaMinus) == 0)
        end
    end

    for i = 1:numberOfVertices
        deltaPlus = edgesLeaving(i, edges)
        @constraint(model, sum(x[i,j] for j in deltaPlus) <= 1)
    end
    
    for i = 1:numberOfVertices
        deltaPlus = edgesLeaving(i, edges)
        deltaMinus = edgesArriving(i, edges)
        for j in deltaPlus
            for k in deltaMinus
                edgeIJ = findall(x->x==(i,j), edges)[1]
                edgeKI = findall(x->x==(k,i), edges)[1]
                @constraint(model, step[i] >= distance[edgeIJ] - distance[edgeKI]
                                            - (2 - x[i,j] - x[k,i]) * M)
                @constraint(model, step[i] >= distance[edgeKI] - distance[edgeIJ]
                                            - (2 - x[i,j] - x[k,i]) * M)
                @constraint(model, p >= step[i])
            end
        end
    end

    for edge in 1:numberOfEdges
        @constraint(model, M >= distance[edge])
    end

    #println(model)
    optimize!(model)

    println("Termination status: ", termination_status(model))
    println("Solve time: ", MOI.get(model, MOI.SolveTime()))
end

main()
