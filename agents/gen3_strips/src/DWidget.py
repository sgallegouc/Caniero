

from PySide2.QtWidgets import *


class DWidget(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.blocks = []
        self.dragging = None
        self.threshold = 25
        self.adjust = 30

    def mousePressEvent(self, event):
        if self.dragging == None:
            pos = event.pos()
            for b in self.blocks:
                if abs(b.x() - pos.x() + self.adjust) < self.threshold and abs(b.y() - pos.y() + self.adjust) < self.threshold:
                    self.dragging = b
                    break
        return super().mousePressEvent(event)
    
    def mouseMoveEvent(self, event):
        if self.dragging != None:
            pos = event.pos()
            self.dragging.setGeometry((pos.x()-25)//10*10, (pos.y()-25)//10*10, 50, 50)
            return super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        self.dragging = None
        return super().mouseReleaseEvent(event)

    def setBlocks(self, blocks):
        self.blocks = blocks

    