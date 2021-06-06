Device = 18F452
Xtal = 4          

DelayMS 20    

All_Digital TRUE  

Declare LCD_DataUs 75       
Declare LCD_RSPin = PORTB.2
Declare LCD_ENPin = PORTB.3
Declare LCD_DTPin = PORTB.4
Declare LCD_Interface = 4
Declare LCD_Lines = 2
Declare LCD_Type 0
   
Dim VarWrite As Word  
Dim VarRead As Word                         
Dim Address As Word      
                    
'TRISC = %00011000   
                           
Symbol SCL = PORTC.3  
Symbol SDA = PORTC.4      
         
Address = 20
VarWrite = 150   

loop:             
    Cls
    VarRead = 0
    Print At 1, 1, "UNEMAT BBG"                    
    I2CIn SDA, SCL, %10100001, Address, [VarWrite] 
    DelayMS 500  
    I2CIn SDA, SCL, %10100000, Address, [VarRead]    
    Print At 2, 1, Dec VarRead
    DelayMS 500 
    GoTo loop
    
    
    

