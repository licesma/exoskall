"""EXO-208: the placed modules read bulb=True but do not draw, and bulb=False ones do
(EXO-203's display-state bug). `Occurrence.isLightBulbOn` will not shift it; Fusion's own
`ShowAllComponentsCmd` does. It runs AFTER this script returns, so the screenshots have to
be a separate bridge call."""
cmd = app.userInterface.commandDefinitions.itemById('ShowAllComponentsCmd')
log('ShowAllComponentsCmd: %s' % (cmd.name if cmd else None))
log('execute -> %s' % cmd.execute())
