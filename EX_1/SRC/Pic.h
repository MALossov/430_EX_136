/*
 * @Description:
 * @Author: MALossov
 * @Date: 2022-04-15 18:26:44
 * @LastEditTime: 2022-04-15 18:58:59
 * @LastEditors: MALossov
 * @Reference:
 */
#ifndef Pic_h_
#define Pic_h_

#define picMode_Left 1
#define picMode_Center 2
#define picMode_Right 3

/*
 * left:
 * =======
 * =======
 * =======
 * x
 *
 * center:
 * =======
 * =======
 * =======
 *    x
 * right:
 * =======
 * =======
 * =======
 *       x
 */
#define picMode_Pic_Left 1
#define picMode_Pic_Center 2
#define picMode_Pic_Right 3

extern const int picAmount;
extern int dockerPicId;

extern unsigned char picSizeInfo[][2];
extern unsigned char Pics[][1024];

#endif // !Pic_h_
