from pycram.external_interfaces import tmc

# Describing
    # TODO: FIX AND PERCEPTION

# count
    # go to
    # percieve
    # SIMILIAR to describe but with a count of entitys

# Guide
    # Ask what to do
    # plan guiding
    # guide
    # end guide

# pick up
    # basic pickup TODO: rework the plan, if not already

# bring to
    # NLP Query
    # drive to action
    # percieve action
    # Pickup action
    # NLP- Query
    # go to action
    # place on action

# deliver
    # drive to action
    # percieve action
    # Pickup action
    # go to action
    # hand over action

# tell
talkingNode = tmc.TextToSpeechPublisher()

# here must be the nlp pipeline implemented TODO

person = "Timo"
talkingNode.say(f"please tell me what you would like me to tell {person}" )

# here must be the nlp pipeline implemented TODO

# OPTIONAL: FInd

def goBackToRefereePose(self):
    # Percieve/find waving human

    # go to waving human with discrependency
        # Nav2 pose with calculation of difference
    raise NotImplementedError("Referee pose cant be determined")