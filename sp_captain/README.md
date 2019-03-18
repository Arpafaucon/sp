# Captain

the captain node is responsible for taking the general orders of the admiral, and converting them into more immediate ones

### Target attribution

`captain` gives to each drone his next target by building an `association array`.

> An 'association' is an array (type: int[num_drones]) 
> that gives for each drone `active_id` the index of the corresponding target id ( and the target coordinates index in 'target_xs' and 'target_ys').
> Thus: the drone `i` must go to
> (target_xs[association[i]], target_ys[association[i]])