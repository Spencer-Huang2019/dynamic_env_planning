from move_dynamic_planning_interface import _move_dynamic_roscpp_initializer


def roscpp_initialize(args):
    # remove __name:= argument
    args2 = [a for a in args if not a.startswith("__name:=")]
    _move_dynamic_roscpp_initializer.roscpp_init("move_dynamic_commander_wrappers", args2)


def roscpp_shutdown():
    _move_dynamic_roscpp_initializer.roscpp_shutdown()
