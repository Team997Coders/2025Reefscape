Automatic Driving
    To source:
    when moving and reaches source, stops moving, finished
    when source is goal moves to source
    Same code for driving to reef and procesor
        Dependancies: alliance, left or right side for reef and source, side of reef
    
    When finished call next action (intake or shoot coral), doesn't mention algae?
        Dependancies: elevator being at correct level, that there is a next action
    checks elevator periodically until it reaches correct level

Automatic Subsystems
    elevator going to sources level:
     goes to source level, checks if there continuosly

    Elevator going to index level:
        Switches from being still to moving to index level?
        When 1st beambrake triggered, starts driving
        When 2nd beambrake triggered, index finished, coral secured

Check Subsystem Status
    elevator goes to target level
    index switches to shooting coral
    starts driving when finished shooting (when 2nd beam break triggers)

Next Action 
    when finished shooting, drives to source then indexes
    when finished indexing holds coral and drives to reef
    while using indexer, doesn't drive

    when at reef 
    elevator at correct level: shoot
    otherwise check if at correct level 

    when at source 
    elevator at correct level: intake 
    otherwise check if at correct level
