% how to use this script
%  mode==1) produces data '2x1000_mat.txt'
%  mode==2) read result from c-code 'ix.txt' and plots

clear, clc, %close all
cd '/wg/stor1/calonder/dev/mpi_kmeans'

mode = 2

if (mode == 1)
   sc = 100;
   X(1:500,:)    = sc*3*randn(500,2)  + repmat(sc*[2,5],500,1);
   X(501:1000,:) = sc*3*randn(500,2)  + repmat(sc*[-3,20],500,1);
   %X = round(X);
   dlmwrite('1000x2_mat.txt',X,'delimiter',' ','precision','%.10e');
   figure, hold on
      plot(X(1:500,1),X(1:500,2),'b.');
      plot(X(501:1000,1),X(501:1000,2),'k.');
else
   X = dlmread('1000x2_mat.txt');
   ix = dlmread('ix.txt');
   CX = dlmread('CX.txt'); % centers
   figure, hold on
      plot(X(1:500,1),X(1:500,2),'b.');
      plot(X(501:1000,1),X(501:1000,2),'g.');
      
   for i=1:length(ix)
      if (i<=500 && ix(i) ~= 0) || (i>500 && ix(i) ~= 1)  % misclustered
         plot(X(i,1),X(i,2),'ro')
      end
   end
   
   plot(CX(1,1),CX(1,2),'db','MarkerSize',10,'LineWidth',3,'MarkerFaceColor','k');
   plot(CX(2,1),CX(2,2),'dg','MarkerSize',10,'LineWidth',3,'MarkerFaceColor','k');
end

   


