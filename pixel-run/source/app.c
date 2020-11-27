/********************************************************************************
  @file     App.c
  @brief    Application functions
  @author   N. Magliola, G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 *******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "kernel/kernel.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DISPLAY_SIZE	       	        8	// Display side number of digits (8x8)
#define RUNNER_POS_INIT                 4



//  UP-LEFT CORNER -----------> (+x)
//  .
//  .           SCREEN
//  .
//  v (+y)



/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief   Updates a matrix of (DISPLAY_SIZE)x(DISPLAY_SIZE) RGBs (3 bytes) and
 *          calls the kernel method that updates the LEDs matrix.
 */
static void updateDisplay(void);

/**
 * @brief   Scrolls down the obstacles. Checks if there was a collision and acts
 *          accordingly. Calls updateDisplay() in the end.
 */
static void scrollObstacles(void);

/**
 * @brief   Generates a new obstacle line.
 */
static kernel_color_t * generateObstacleLine(void);

/**
 * @brief   Moves player to the right. Calls updateDisplay() in the end.
 */
static void moveRight(void);

/**
 * @brief   Moves player to the left. Calls updateDisplay() in the end.
 */
static void moveLeft(void);

/**
 * @brief   Toggles the variable gameContext.gamePaused.
 */
static void togglePause(void);

/**
 * @brief   Checks if there was a collision. If so, calls game-over animation
 *          resets the game and pauses it.
 */
static bool checkCollision(void);

/**
 * @brief   Resets game values to initial ones.
 */
static void gameInit(void);

/**
 * @brief   Resets game values to initial ones.
 */
static void gameOverUpdate(void);

/**
 * @brief   Resets game values to initial ones.
 */
static void scoreManager(void);


/*******************************************************************************
 * VARIABLES TYPES DEFINITIONS
 ******************************************************************************/

typedef enum
{
    DIFF_0,
    DIFF_1,
    DIFF_2,
    DIFF_3,
    DIFF_NUM
} difficulty_t;


typedef struct
{
    uint16_t            score;                   
    uint8_t             runnerPos;                          // runner´s position (0 to DISPLAY_SIZE - 1)
    bool                gamePaused;                         // pause status
    difficulty_t        difficulty;                         // game difficulty
    uint8_t             tileOfBlocks;                       // last tile extracted from blocksPattern
    bool                blankTile;
    kernel_color_t      board[DISPLAY_SIZE][DISPLAY_SIZE];  // matrix containing colours of each led in the display
    bool                gameOverAnimation;
    uint8_t             gameOverIndex;
} pixelrun_context_t;


typedef kernel_color_t pixelrun_block_matrix_t[7][DISPLAY_SIZE];

/*******************************************************************************
 * PRIVATE VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static pixelrun_context_t       gameContext;                // Game context and data
static pixelrun_block_matrix_t  blocksPattern = 
{
    {KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE},
    {KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE},
    {KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK},
    {KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE},
    {KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLACK},
    {KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE},
    {KERNEL_BLACK, KERNEL_BLACK, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLUE, KERNEL_BLACK, KERNEL_BLACK},
};          // Matrix to draw tiles of obstacles from

static uint16_t scrollTime[DIFF_NUM] = {10000, 5000, 2500, 1250};

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Called once at the beginning of the program */
void appInit (void)
{
    kernelInit();
    gameInit();
}

/* Called repeatedly in an infinite loop */
void appRun (void)
{
    kernel_event_t ev = kernelGetNextEvent();
    if (ev.id != KERNEL_NO_EVENT)
    {
        switch (ev.id)
        {
            case KERNEL_TIMEOUT:
            { 
                if (gameContext.gameOverAnimation)
                {
                    gameOverUpdate();
                }
                else
                {
                    if(!gameContext.gamePaused) 
                        scrollObstacles();
                } 
            } break;
            case KERNEL_RIGHT: if(!gameContext.gamePaused && !gameContext.gameOverAnimation) moveRight(); break;
            case KERNEL_LEFT: if(!gameContext.gamePaused && !gameContext.gameOverAnimation) moveLeft(); break;
            case KERNEL_ENTER: if(!gameContext.gameOverAnimation) togglePause(); break;
        }
        
        if (checkCollision() && !gameContext.gameOverAnimation)
        {
            gameContext.gameOverAnimation = true;
            gameOverUpdate();
        }
        else if (ev.id == KERNEL_TIMEOUT && !gameContext.gameOverAnimation)
        {
            scoreManager();
        }
        
        updateDisplay();
    }
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void gameInit(void)
{
    gameContext.runnerPos = RUNNER_POS_INIT;
    gameContext.difficulty = DIFF_0;
    gameContext.score = 0;
    gameContext.gamePaused = false;
    
    gameContext.tileOfBlocks = 3;
    gameContext.blankTile = true;
    
    for (uint8_t i=0; i<DISPLAY_SIZE; i++)
    {
        if (!(i % 2))
        {
            for (uint8_t j=0; j<DISPLAY_SIZE; j++)
            {
                gameContext.board[i][j] = blocksPattern[i/2][j];

            }
        }
        else
        {
            for (uint8_t j=0; j<DISPLAY_SIZE; j++)
            {
                gameContext.board[0][j] = KERNEL_BLACK;
            }
        }
    }

    gameContext.gameOverAnimation = false;
    gameContext.gameOverIndex = 0;

    updateDisplay();
    kernelStartTimer(scrollTime[DIFF_0], false);
}

void togglePause(void)
{
    gameContext.gamePaused = !gameContext.gamePaused;
}

void moveLeft(void)
{
    if (gameContext.runnerPos > 0 )
    {
        gameContext.runnerPos--;
    }
}

void moveRight(void)
{
    if (gameContext.runnerPos < (DISPLAY_SIZE-1) )
    {
        gameContext.runnerPos++;
    }
}

void scrollObstacles(void)
{
    for (uint8_t i=7; i>0; i--)
    {
        for (uint8_t j=0; j<DISPLAY_SIZE; j++)
        {
            gameContext.board[i][j] = gameContext.board[i-1][j];
        }
    }

    if (gameContext.blankTile)
    {
        for (uint8_t j=0; j<DISPLAY_SIZE; j++)
        {
            gameContext.board[0][j] = KERNEL_BLACK;
        }
    }
    else
    {
        kernel_color_t * newObstaclesTile = generateObstacleLine();
        for (uint8_t j=0; j<DISPLAY_SIZE; j++)
        {
            gameContext.board[0][j] = newObstaclesTile[j];
        }
    }

    gameContext.blankTile = !gameContext.blankTile;
}

kernel_color_t * generateObstacleLine(void)
{
	kernel_color_t * ret;
	if (gameContext.tileOfBlocks < 7)
	{
	  ret = blocksPattern[gameContext.tileOfBlocks++];
	}
	else
	{
	  gameContext.tileOfBlocks = 0;
	  ret = blocksPattern[gameContext.tileOfBlocks++];
	}

	return ret;
}

void updateDisplay(void)
{
    kernelDisplay(gameContext.board, gameContext.runnerPos);
}

bool checkCollision(void)
{
//    return (gameContext.board[DISPLAY_SIZE-1][gameContext.runnerPos] != KERNEL_BLACK); // checks color in runner's position
	return false;
}

void gameOverUpdate(void)
{
    if (gameContext.gameOverIndex < DISPLAY_SIZE*DISPLAY_SIZE)
    {
        kernelStartTimer(50, false);
        *((kernel_color_t*)gameContext.board + gameContext.gameOverIndex++) = KERNEL_RED;
    }
    else
    {
        gameInit();
    }
}

void scoreManager(void)
{
    gameContext.score++;

    if (gameContext.score < 10)
    {
        kernelStartTimer(scrollTime[DIFF_0], false);
    }
    else if (gameContext.score < 30)
    {
        kernelStartTimer(scrollTime[DIFF_1], false);
    }
    else if (gameContext.score < 70)
    {
        kernelStartTimer(scrollTime[DIFF_2], false);
    }
    else if (gameContext.score < 150)
    {
        kernelStartTimer(scrollTime[DIFF_3], false);
    }
}
/*******************************************************************************
 ******************************************************************************/
