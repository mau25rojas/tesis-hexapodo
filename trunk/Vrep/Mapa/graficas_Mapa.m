%Dibujo de mapas

mapa = load('Mapa_Caso14.txt');

imagesc(mapa');            %# Create a colored plot of the matrix values
colormap(flipud(gray));  %# Change the colormap to gray (so higher values are
                         %#   black and lower values are white)
set(gca,'XTick',0:100)
set(gca,'XTick',[1 25 50 75 100])
set(gca,'YTick',0:20)
set(gca,'YTick',[1 10 20])
axis image
print -depsc M14.eps

