package frc.robot.exercices;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

import java.util.OptionalInt;
import java.util.function.IntPredicate;
import java.util.function.IntUnaryOperator;

/**
 * MODE D'EMPLOI
 * 
 * Chaque exercice vient sous la forme d'une méthode à implémenter et d'un test
 * qui vérifie que l'exerice est réussi.
 * 
 * 0) Créer une nouvelle branche pour l'exercice (vous ne ferez jamais de Pull Request avec celle là).
 * 1) Décommenter le contenu du test du premier exercice pas encore réussi.
 *    Ex: void whenFirst().
 * 2) Étudier la consigne de l'exercice, puis les tests associés pour bien comprendre.
 * 3) Modifier la méthode de l'exercice pour atteindre l'objectif de l'exercice.
 *    Ex: OptionalInt first(int[] array).
 * 4) Éxécuter les tests avec [Ctrl] + [Shift] + [P] puis "WPILib: Test Robot Code".
 * 5) Vérifier les erreurs en ouvrant le fichier HTML listé dans le terminal dans un
 *    navigateur internet (il faut copier / coller le lien à la main).
 * 6) S'il y a une erreur, retourner au 3).
 * 7) S'il n'y a pas d'erreur, continuer au 1).
 * 
 * Ressources:
 * 
 * - Aide mémoire Java pour les tableaux: https://docs.google.com/document/d/1yh1mrDhENs3qkGIoI7GcS1PqAGfUkODylAafZy3zXUA/edit
 * - Boucles for: https://www.w3schools.com/java/java_for_loop.asp
 */
public class ArrayExercices {
    static final double EPSILON = 0.0000000001;

    // Retourne la valeur du premier élément ou empty() si le tableau est vide ou null.
    OptionalInt first(int[] array) {
        return OptionalInt.empty();
    }

    @Test
    void whenFirst() {
        // assertEquals(OptionalInt.empty(), first(null));
        // assertEquals(OptionalInt.empty(), first(new int[0]));

        // assertEquals(OptionalInt.of(7), first(new int[]{ 7 }));
        // assertEquals(OptionalInt.of(3), first(new int[]{ 3, 7 }));
    }

    // Retourne la valeur du dernier élément ou empty() si le tableau est vide ou null.
    OptionalInt last(int[] array) {
        return OptionalInt.empty();
    }

    @Test
    void whenLast() {
        // assertEquals(OptionalInt.empty(), last(null));
        // assertEquals(OptionalInt.empty(), last(new int[0]));

        // assertEquals(OptionalInt.of(7), last(new int[]{ 7 }));
        // assertEquals(OptionalInt.of(7), last(new int[]{ 3, 7 }));
    }

    // Retourne l'indice de la première occurence de l'élément dans le tableau,
    // ou -1 si l'élément n'existe pas dans le tableau, ou si le tableau est null.
    int findFirst(int[] array, int elt) {
        return -2;
    }

    @Test
    void whenFindFirst() {
        // assertEquals(-1, findFirst(null, 3));
        // assertEquals(-1, findFirst(new int[0], 3));

        // assertEquals(-1, findFirst(new int[]{ 3, 7 }, 8));

        // assertEquals(0, findFirst(new int[]{ 3, 7 }, 3));
        // assertEquals(1, findFirst(new int[]{ 3, 7 }, 7));
        // assertEquals(0, findFirst(new int[]{ 3, 7, 3 }, 3));
    }

    
    // Retourne l'indice de la dernière occurence de l'élément dans le tableau,
    // ou -1 si l'élément n'existe pas dans le tableau, ou si le tableau est null.
    int findLast(int[] array, int elt) {
        return -2;
    }

    @Test
    void whenFindLast() {
        // assertEquals(-1, findLast(null, 3));
        // assertEquals(-1, findLast(new int[0], 3));

        // assertEquals(-1, findLast(new int[]{ 3, 7 }, 8));
        
        // assertEquals(0, findLast(new int[]{ 3, 7 }, 3));
        // assertEquals(1, findLast(new int[]{ 3, 7 }, 7));
        // assertEquals(2, findLast(new int[]{ 3, 7, 3 }, 3));
    }

    // Retourne la plus petite valeur ou empty() si le tableau est vide ou null.
    OptionalInt min(int[] array) {
        return OptionalInt.empty();
    }

    @Test
    void whenMin() {
        // assertEquals(OptionalInt.empty(), min(null));
        // assertEquals(OptionalInt.empty(), min(new int[0]));

        // assertEquals(OptionalInt.of(7), min(new int[]{ 7 }));
        // assertEquals(OptionalInt.of(3), min(new int[]{ 3, 7 }));
        // assertEquals(OptionalInt.of(3), min(new int[]{ 7, 3 }));
        // assertEquals(OptionalInt.of(-7), min(new int[]{ -7, 3 }));
        // assertEquals(OptionalInt.of(3), min(new int[]{ 7, 3, 6 }));
    }

    // Retourne la somme des éléments ou 0 si le tableau est vide ou null.
    int sum(int[] array) {
        return -1;
    }

    @Test
    void whenSum() {
        // assertEquals(0, sum(null));
        // assertEquals(0, sum(new int[0]));

        // assertEquals(7, sum(new int[]{ 7 }));
        // assertEquals(10, sum(new int[]{ 3, 7 }));
        // assertEquals(10, sum(new int[]{ 7, 3 }));
        // assertEquals(16, sum(new int[]{ 7, 3, 6 }));
    }

    // Retourne la moyenne des éléments ou 0 si le tableau est vide ou null.
    double avg(int[] array) {
        return -1;
    }

    @Test
    void whenAvg() {
        // assertEquals(0.0, avg(null));
        // assertEquals(0.0, avg(new int[0]));

        // assertEquals(7.0, avg(new int[]{ 7 }));
        // assertEquals(5.0, avg(new int[]{ 3, 7 }));
        // assertEquals(5.0, avg(new int[]{ 7, 3 }));
        // assertEquals(3.5, avg(new int[]{ 7, 3, 3, 1 }), EPSILON);
    }

    // Retourne un tableau de n éléments avec le même élément
    // Retourne une erreur lorsque n est négatif.
    int[] fill(int n, int elt) {
        if (n < 0) {
            throw new IllegalArgumentException("n doit être nul ou positif");
        }
        return null;
    }

    @Test
    void whenFill() {
        // assertThrows(IllegalArgumentException.class, () -> fill(-1, 3));

        // assertArrayEquals(new int[]{}, fill(0, 3));

        // assertArrayEquals(new int[]{ 3 }, fill(1, 3));
        // assertArrayEquals(new int[]{ 3, 3, 3, 3, 3 }, fill(5, 3));
    }

    // Retourne un tableau de n éléments. Le premier élément est firstElt, et les autres augmentent de 1 à chaque fois.
    // Retourne une erreur lorsque n est négatif.
    int[] serie(int n, int firstElt) {
        return null;
    }

    @Test
    void whenSerie() {
        // assertThrows(IllegalArgumentException.class, () -> serie(-1, 3));

        // assertArrayEquals(new int[]{}, serie(0, 3));

        // assertArrayEquals(new int[]{ 3 }, serie(1, 3));
        // assertArrayEquals(new int[]{ 3, 4, 5, 6, 7 }, serie(5, 3));
    } 

    // Prend un tableau à 2 élément et interverti les 2 élément.
    // Retourne une erreur lorsque le tableau n'a pas 2 éléments ou est null.
    void switchElements(int[] pair) {

    }

    @Test
    void whenSwitchElements() {
        // assertThrows(IllegalArgumentException.class, () -> switchElements(null));
        // assertThrows(IllegalArgumentException.class, () -> switchElements(new int[0]));
        // assertThrows(IllegalArgumentException.class, () -> switchElements(new int[1]));
        // assertThrows(IllegalArgumentException.class, () -> switchElements(new int[3]));

        // var array1 = new int[]{ 7, 3 };
        // switchElements(array1);
        // assertArrayEquals(new int[]{ 3, 7 }, array1);

        // var array2 = new int[]{ 3, 7 };
        // switchElements(array2);
        // assertArrayEquals(new int[]{ 7, 3 }, array2);
    }

    // Retourne un nouveau tableau avec les éléments dans l'ordre inversé.
    // Retourne null si le tableau est null.
    int[] reverse(int[] array) {
        return null;
    }

    @Test
    void whenReverse() {
        // assertNull(reverse(null));

        // assertArrayEquals(new int[0], reverse(new int[0]));

        // assertArrayEquals(new int[] { 3 }, reverse(new int[] { 3 }));

        // var array1 = new int[] { 3, 7 };
        // assertArrayEquals(new int[] { 7, 3 }, reverse(array1));
        // assertArrayEquals(new int[] { 3, 7 }, array1);

        // var array2 = new int[] { 3, 7, 2, 9 };
        // assertArrayEquals(new int[] { 9, 2, 7, 3 }, reverse(array2));
        // assertArrayEquals(new int[] { 3, 7, 2, 9 }, array2);
    }

    // Retourne un nouveau tableau avec tous les éléments du premier, puis du deuxième, dans le même ordre.
    // Retourne null si les 2 tableaux sont null. Ignore un tableau si l'un des 2 est null.
    int[] concat(int[] array1, int[] array2) {
        return null;
    }

    @Test
    void whenConcat() {
        // assertNull(concat(null, null));

        // assertArrayEquals(new int[0], concat(new int[0], null));
        // assertArrayEquals(new int[0], concat(null, new int[0]));

        // assertArrayEquals(new int[] { 3, 7 }, concat(new int[] { 3, 7 }, null));
        // assertArrayEquals(new int[] { 3, 7 }, concat(null, new int[] { 3, 7 }));
        // assertArrayEquals(new int[] { 3, 7 }, concat(new int[] { 3, 7 }, new int[0]));
        // assertArrayEquals(new int[] { 3, 7 }, concat(new int[0], new int[] { 3, 7 }));

        // var array11 = new int[] { 3, 7 };
        // var array12 = new int[] { 10 };
        // assertArrayEquals(new int[] { 3, 7, 10 }, concat(array11, array12));
        // assertArrayEquals(new int[] { 3, 7 }, array11);
        // assertArrayEquals(new int[] { 19, -1 }, array12);
        // assertArrayEquals(new int[] { 10, 3, 7 }, concat(array12, array11));
        // assertArrayEquals(new int[] { 3, 7 }, array11);
        // assertArrayEquals(new int[] { 19, -1 }, array12);

        // var array21 = new int[] { 3, 7, -1, 33 };
        // var array22 = new int[] { 10, -9, 100 };
        // assertArrayEquals(new int[] { 3, 7, -1, 33, 10, -9, 100 }, concat(array21, array22));
        // assertArrayEquals(new int[] { 3, 7, -1, 33 }, array21);
        // assertArrayEquals(new int[] { 10, -9, 100 }, array22);
        // assertArrayEquals(new int[] { 10, -9, 100 , 3, 7, -1, 33 }, concat(array22, array21));
        // assertArrayEquals(new int[] { 3, 7, -1, 33 }, array21);
        // assertArrayEquals(new int[] { 10, -9, 100 }, array22);
    }

    // Retourne un nouveau tableau avec les mêmes éléments dans le même ordre excepté l'élément à l'indice eltIndex.
    // Retourne une erreur si eltIndex est négatif ou si le tableau n'a pas au moins (eltIndex + 1) éléments.
    int[] remove(int[] array, int eltIndex) {
        return null;
    }

    @Test
    void whenRemove() {
        // assertThrows(IllegalArgumentException.class, () -> remove(null, 0));
        // assertThrows(IllegalArgumentException.class, () -> remove(new int[0], 0));
        // assertThrows(IllegalArgumentException.class, () -> remove(new int[3], 3));
        // assertThrows(IllegalArgumentException.class, () -> remove(new int[3], -1));

        // assertArrayEquals(new int[0], remove(new int[] { 3 }, 0));

        // var array1 = new int[] { 3, 7 };
        // assertArrayEquals(new int[] { 7 }, remove(array1, 0));
        // assertArrayEquals(new int[] { 3, 7 }, array1);
        // assertArrayEquals(new int[] { 3 }, remove(array1, 1));
        // assertArrayEquals(new int[] { 3, 7 }, array1);

        // var array2 = new int[] { 3, 7, 10, -1 };
        // assertArrayEquals(new int[] { 7, 10, -1 }, remove(array2, 0));
        // assertArrayEquals(new int[] { 3, 7, 10, -1 }, array2);
        // assertArrayEquals(new int[] { 3, 10, -1 }, remove(array2, 1));
        // assertArrayEquals(new int[] { 3, 7, 10, -1 }, array2);
        // assertArrayEquals(new int[] { 3, 7, 10 }, remove(array2, 3));
        // assertArrayEquals(new int[] { 3, 7, 10, -1 }, array2);
    }

    // Retourne vrai si le tableau est un palindrome, faux sinon.
    // Retourne une erreur si le tableau est null.
    boolean isPalindrome(int[] array) {
        return true;
    }

    @Test
    void whenIsPalindrome() {
        // assertThrows(IllegalArgumentException.class, () -> isPalindrome(null));

        // assertTrue(isPalindrome(new int[0]));
        // assertTrue(isPalindrome(new int[] { 7 }));
        // assertTrue(isPalindrome(new int[] { 7, 7 }));
        // assertTrue(isPalindrome(new int[] { 7, 3, 7 }));
        // assertTrue(isPalindrome(new int[] { 7, 3, 3, 7 }));
        // assertTrue(isPalindrome(new int[] { 7, 3, -1, 3, 7 }));

        // assertFalse(isPalindrome(new int[] { 7, 3 }));
        // assertFalse(isPalindrome(new int[] { 3, 7 }));
        // assertFalse(isPalindrome(new int[] { 3, 7, 7, 4 }));
        // assertFalse(isPalindrome(new int[] { 3, 7, 10, 3 }));
        // assertFalse(isPalindrome(new int[] { 3, 7, -1, 7, 4 }));
        // assertFalse(isPalindrome(new int[] { 3, 7, -1, 10, 3 }));
    }

    // Retourne un nouveau tableau qui contient uniquement les éléments qui
    // respectent la condition, dans le même ordre.
    // Retourne null si le tableau est null.
    // Retourne une erreur si la condition est nulle.
    int[] filter(int[] array, IntPredicate condition) {
        return null;
    }

    @Test
    void whenFilter() {
        // IntPredicate isPositive =  (x) -> x >= 0;
        // IntPredicate isPair =      (x) -> (x % 2) == 0;
        // IntPredicate alwaysTrue =  (x) -> true;
        // IntPredicate alwaysFalse = (x) -> false;

        // assertNull(filter(null, isPositive));

        // assertThrows(IllegalArgumentException.class, () -> filter(null, null));
        // assertThrows(IllegalArgumentException.class, () -> filter(new int[0], null));

        // assertArrayEquals(new int[0], filter(new int[0], isPositive));

        // var array1 = new int[] { 3, -1, 4, -2 };
        // assertArrayEquals(new int[] { 3, 4 }, filter(array1, isPositive));
        // assertArrayEquals(new int[] { 3, -1, 4, -2 }, array1);

        // assertArrayEquals(new int[] { 4, -2 }, filter(array1, isPair));
        // assertArrayEquals(new int[] { 3, -1, 4, -2 }, array1);

        // assertArrayEquals(new int[] { 3, -1, 4, -2 }, filter(array1, alwaysTrue));
        // assertArrayEquals(new int[] { 3, -1, 4, -2 }, array1);

        // assertArrayEquals(new int[0], filter(array1, alwaysFalse));
        // assertArrayEquals(new int[] { 3, -1, 4, -2 }, array1);
    }

    // Retourne un nouveau tableau qui contient, pour chaque élément du tableau, l'élément transformé par l'opération.
    // Retourne null si le tableau est null.
    // Retourne une erreur si l'opération est nulle.
    int[] transform(int[] array, IntUnaryOperator operation) {
        return null;
    }

    @Test
    void whenTransform() {
        // IntUnaryOperator plus3 =  (x) -> x + 3;
        // IntUnaryOperator square = (x) -> x * x;

        // assertNull(transform(null, plus3));
        
        // assertThrows(IllegalArgumentException.class, () -> transform(null, null));
        // assertThrows(IllegalArgumentException.class, () -> transform(new int[0], null));

        // assertArrayEquals(new int[0], transform(new int[0], plus3));

        // var array1 = new int[] { 3, -1, 4, -2 };
        // assertArrayEquals(new int[] { 6, 2, 7, 1 }, transform(array1, plus3));
        // assertArrayEquals(new int[] { 3, -1, 4, -2 }, array1);

        // assertArrayEquals(new int[] { 9, 1, 16, 4 }, transform(array1, square));
        // assertArrayEquals(new int[] { 3, -1, 4, -2 }, array1);
    }    
}
