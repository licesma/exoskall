"""EXO-225: is there a NON-destructive section analysis in this Fusion's API?"""
log('design attrs with "anal": %s' % [a for a in dir(design) if 'anal' in a.lower()])
log('root attrs with "anal"/"sect": %s'
    % [a for a in dir(design.rootComponent) if 'anal' in a.lower() or 'sect' in a.lower()])
log('adsk.fusion names with "Section": %s'
    % [a for a in dir(adsk.fusion) if 'Section' in a])
try:
    an = design.rootComponent.analyses
    log('rootComponent.analyses -> %s, count %d' % (an, an.count))
    log('  create methods: %s' % [a for a in dir(an) if a.startswith('create') or a.startswith('add')])
except Exception as e:
    log('rootComponent.analyses: %s' % e)
try:
    an = design.analyses
    log('design.analyses -> %s' % an)
except Exception as e:
    log('design.analyses: %s' % e)
log('Occurrences.addNewComponentCopy present: %s'
    % hasattr(design.rootComponent.occurrences, 'addNewComponentCopy'))
