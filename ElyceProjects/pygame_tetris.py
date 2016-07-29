import pygame
import sys
from pygame.locals import *
import random

pygame.init()

class Tetris():
	def __init__(self):
		#INIT ALL THE THINGS
		self.gameOver = False
		self.score = 0
		self.fps = 60
		self.fpsClock = pygame.time.Clock()
		self.clock_counter = 0
		self.cols = 10
		self.rows = 15
		self.margin = 20
		self.cellmargin = 1
		self.cell_size = 20
		self.boardwidth = 2 * self.margin + self.cols * self.cell_size
		self.boardheight = 2 * self.margin + self.rows * self.cell_size
		self.black = (0,0,0)
		self.white = (255,255,255)
		self.red = (255,0,0)
		self.green = (0,255,0)
		self.blue = (0,0,255)
		self.orange = (255,165,0)
		self.cyan = (0,255,255)
		self.magenta = (255,0,255)
		self.yellow = (255,255,0)
		self.pink = (255,105, 180)
		
		self.empty_color = self.blue
		self.board = [([self.empty_color] * self.cols) for row in range(self.rows)]


		# # #test parameters for the board
		# self.board[0][0] = self.red
		# self.board[0][self.cols-1] = self.white
		# self.board[self.rows-1][0] = self.green
		# self.board[self.rows-1][self.cols-1] = self.red

		self.tetrisGenShapes()
		self.newFallingPiece()

		self.surface = pygame.display.set_mode((self.boardwidth,self.boardheight))

	def getCellBounds(self,row, col):
		gridWidth = self.boardwidth - 2*self.margin
		gridHeight = self.boardheight - 2*self.margin
		x0 = self.margin + gridWidth * col / self.cols
		x1 = self.margin + gridWidth * (col+1) / self.cols
		y0 = self.margin + gridHeight * row / self.rows
		y1 = self.margin + gridHeight * (row+1) / self.rows
		cell_width = x1-x0
		cell_height = y1-y0
		return (x0, y0, cell_width,cell_height)		

	def newFallingPiece(self):
		pieceindex = random.randint(0,len(self.tetrisPieces)-1)
		self.fallingPiece = self.tetrisPieces[pieceindex]
		self.fallingPieceColor = self.tetrisPieceColors[pieceindex]
		self.fp_cols = len(self.fallingPiece)
		self.fp_row = 0
		self.fp_col = self.cols//2 - (self.fp_cols +1)//2

	def isLegal(self):
		for row in range(len(self.fallingPiece)):
			for col in range(len(self.fallingPiece[0])):
				if self.fallingPiece[row][col] == True:
					cellx = col + self.fp_col 
					celly = row + self.fp_row
					if cellx < 0 or cellx >= len(self.board[0]):
						return False
					if celly < 0 or celly >= len(self.board):
						return False
					if self.board[celly][cellx] != self.empty_color:
						return False
		return True

	def moveFallingPiece(self, drow, dcol):
		self.fp_row += drow
		self.fp_col += dcol
		if self.isLegal() == False:
			self.fp_row -= drow
			self.fp_col -= dcol
			return False
		return True

	def rotateFallingPiece(self):
		empty_list = [([None] * len(self.fallingPiece)) for col in range(len(self.fallingPiece[0]))]
		for y in range(len(self.fallingPiece)):
			for x in range(len(self.fallingPiece[0])):
				empty_list[x][y] = self.fallingPiece[y][x]
		newPiece = [([None] * len(self.fallingPiece)) for col in range(len(self.fallingPiece[0]))]
		for newx in range(len(empty_list)):
			for newy in range(len(empty_list[0])- 1, -1, -1):
				newPiece[newx][len(newPiece[0]) -1 - newy] = empty_list[newx][newy]
		oldPiece = self.fallingPiece
		self.fallingPiece = newPiece
		if self.isLegal() == False:
			self.fallingPiece = oldPiece

	def placeFallingPiece(self):
		for col in range(len(self.fallingPiece[0])):
			for row in range(len(self.fallingPiece)):
				if self.fallingPiece[row][col] == True:
					self.board[row + self.fp_row][col + self.fp_col] = self.fallingPieceColor

	def isFull(self, y):
		for x in range(len(self.board[0])):
			if self.board[y][x] == self.empty_color:
				return False
		return True

	def removeFullRows(self):
		counter = 0
		newBoard =[([self.empty_color] * self.cols) for row in range(self.rows)]
		newBoardIndex = len(newBoard) -1
		for oldRow in range(len(self.board) -1, -1, -1):
			if self.isFull(oldRow) == False:
				for x in range(len(self.board[0])):
					newBoard[newBoardIndex][x] = self.board[oldRow][x]
				newBoardIndex -= 1
			else:
				counter += 1
		self.board = newBoard
		self.score += counter

	def drawGame(self):
		pygame.draw.rect(self.surface,self.orange, (0,0, self.boardwidth,self.boardheight), 0)
		font = pygame.font.Font('freesansbold.ttf',12)
		textsurface = font.render('Score = %d' %(self.score), True, self.black, self.orange)
		textrect = textsurface.get_rect()
		textrect.center = (self.margin * 2, self.margin / 2)
		self.surface.blit(textsurface,textrect)

	def drawFallingPiece(self):
		for row in range(len(self.fallingPiece)):
			for col in range(len(self.fallingPiece[0])):
				if self.fallingPiece[row][col] == True:
					self.drawCell(row + self.fp_row, col + self.fp_col, self.fallingPieceColor)

	def drawBoard(self):
		for row in range(self.rows):
			for col in range(self.cols):
				self.drawCell(row,col, self.board[row][col])

	def drawCell(self, row, col, color):
		m = self.cellmargin
		(x0, y0, cell_width,cell_height) = self.getCellBounds(row, col)
		pygame.draw.rect(self.surface, self.black, (x0,y0,cell_width,cell_height))
		pygame.draw.rect(self.surface, color, (x0+m,y0+m,cell_width-(2*m),cell_height-(2*m)),2)

	def drawEnd(self):
		font = pygame.font.Font('freesansbold.ttf',32)
		textsurface = font.render('Game over :P', True, self.green, self.white)
		textrect = textsurface.get_rect()
		textrect.center = (self.boardwidth/2, self.boardheight/2)
		self.surface.blit(textsurface,textrect)

	def tetrisGenShapes(self):
		  #Seven "standard" pieces (tetrominoes)
		self.iPiece = [
		[ True,  True,  True,  True]
		]

		self.jPiece = [
		[ True, False, False ],
		[ True, True,  True]
		]

		self.lPiece = [
		[ False, False, True],
		[ True,  True,  True]
		]

		self.oPiece = [
		[ True, True],
		[ True, True]
		]

		self.sPiece = [
		[ False, True, True],
		[ True,  True, False ]
		]

		self.tPiece = [
		[ False, True, False ],
		[ True,  True, True]
		]

		self.zPiece = [
		[ True,  True, False ],
		[ False, True, True]
		]

		self.tetrisPieces = [ self.iPiece, self.jPiece, self.lPiece, self.oPiece, self.sPiece, self.tPiece, self.zPiece ]
		self.tetrisPieceColors = [ self.red, self.yellow , self.magenta, self.pink, self.cyan, self.green, self.orange ]

	def keyBindings(self, event):
		if self.gameOver == False:	
			if event.key == K_UP:
				self.rotateFallingPiece()
			elif event.key == K_DOWN:
				self.moveFallingPiece(1,0)
			elif event.key == K_LEFT:
				self.moveFallingPiece(0,-1)
			elif event.key == K_RIGHT:
				self.moveFallingPiece(0,1)

	def step(self):
		if self.gameOver == False:
			if (self.clock_counter % 60) == 0:

				if self.moveFallingPiece(1,0) == False:
					self.placeFallingPiece()
					self.newFallingPiece()
					if self.isLegal() == False:
						self.gameOver = True
			self.clock_counter += 1
			for y in range(len(self.board)-1, -1, -1):
				if self.isFull(y) == True:
					self.removeFullRows()

	def runTetris(self):
		while True:
			self.drawGame()
			self.drawBoard()
			self.drawFallingPiece()
			for event in pygame.event.get():
				if event.type == QUIT:
					pygame.quit()
					sys.exit()
				elif event.type == KEYDOWN:
					self.keyBindings(event)
			self.step()
			if self.gameOver == True:
				self.drawEnd()
			pygame.display.update()
			self.fpsClock.tick(self.fps)


game = Tetris()
game.runTetris()