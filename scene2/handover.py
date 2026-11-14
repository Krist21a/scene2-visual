"""Release interlock for a two-hand transfer; a plan alone is not possession.

Motion planning must supply a checked rendezvous. This state machine authorizes
donor release only after fresh receiver contact and three visual confirmations.
It never opens a gripper or changes the possession context by itself.
"""
import time
from .control import Stop


class Handover:
    def __init__(self, donor, receiver, log, clock=time.monotonic):
        if donor not in (0,1) or receiver != 1-donor:
            raise ValueError('Handover needs two different arms')
        self.donor,self.receiver,self.log,self.clock=donor,receiver,log,clock
        self.state='holding';self.stable=0;self.last_stamp=None;self.verified_at=None
        self.payload=None
        self.event()

    def event(self, **evidence):
        self.log('handover_event',state=self.state,donor=self.donor,receiver=self.receiver,**evidence)

    def receiver_closed(self):
        if self.state!='holding':raise Stop('Invalid handover receiver-close transition')
        self.state='confirming';self.event()

    def observe(self, stamp, grip, reference, visible_points, payload):
        if self.state not in ('confirming','confirmed'):
            raise Stop('Handover is not awaiting receiver confirmation')
        if self.last_stamp is not None and stamp<=self.last_stamp:
            self.abort('Repeated or out-of-order receiver image')
        self.last_stamp=stamp
        valid=(.02<grip<.97 and reference-grip>.06 and visible_points>=12 and payload is not None)
        if not valid:self.abort('Receiver contact or visual possession is missing')
        self.stable+=1
        if self.stable>=3:
            self.state='confirmed';self.payload=payload;self.verified_at=self.clock()
        self.event(stamp=stamp,grip=grip,visible_points=visible_points,stable=self.stable)

    def authorize_open(self, side):
        if side!=self.donor:return
        if self.state!='confirmed' or self.verified_at is None or self.clock()-self.verified_at>1.:
            self.abort('Donor release requires fresh confirmed receiver possession')

    def begin_release(self):
        self.authorize_open(self.donor)
        self.state='releasing';self.event()

    def transferred(self):
        if self.state!='releasing':self.abort('Transfer cannot complete before authorized donor opening')
        self.state='transferred';self.event()

    def abort(self, reason):
        self.state='failed';self.event(reason=reason,retain_donor=True,requires_scene_reset=True)
        raise Stop('Handover stopped with donor retained: '+reason)
