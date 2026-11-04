root=design.rootComponent
for i in range(root.occurrences.count):
    o=root.occurrences.item(i)
    bb=o.boundingBox
    log('%-40s visible=%s bodies=%d children=%d bb=(%.0f,%.0f,%.0f)-(%.0f,%.0f,%.0f)'%(
        o.component.name,o.isLightBulbOn,o.bRepBodies.count,o.childOccurrences.count,
        bb.minPoint.x*10,bb.minPoint.y*10,bb.minPoint.z*10,bb.maxPoint.x*10,bb.maxPoint.y*10,bb.maxPoint.z*10))
